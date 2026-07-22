// Copyright 2026 Universal Field Robots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rws/action_handler.hpp"

#include <chrono>
#include <random>
#include <utility>

#include "rws/translate.hpp"

namespace rws
{

using namespace std::chrono_literals;

namespace
{

constexpr const char * kFeedbackSuffix = "_FeedbackMessage";
constexpr const char * kCancelGoalSrv = "action_msgs/srv/CancelGoal";

rclcpp::Logger logger() { return rclcpp::get_logger("rws_action_handler"); }

void send_json(const std::function<void(std::string & msg)> & send, json msg)
{
  std::string s = msg.dump();
  if (send) {
    send(s);
  }
}

json random_uuid()
{
  std::random_device rd;
  std::uniform_int_distribution<int> dist(0, 255);
  json uuid = json::array();
  for (int i = 0; i < 16; i++) {
    uuid.push_back(dist(rd));
  }
  return uuid;
}

json make_result_error(const json & id, const std::string & action, const std::string & error)
{
  return {
    {"id", id},           {"op", "action_result"}, {"action", action},
    {"values", json::object()}, {"status", 0},     {"result", false},
    {"error", error},
  };
}

}  // namespace

std::string normalize_action_type(const std::string & type)
{
  auto first = type.find('/');
  if (first != std::string::npos && type.find('/', first + 1) == std::string::npos) {
    return type.substr(0, first) + "/action/" + type.substr(first + 1);
  }
  return type;
}

ActionHandler::ActionHandler(
  std::shared_ptr<NodeInterface<>> node, std::function<void(std::string & msg)> send_callback)
: node_(node), send_(send_callback), registry_(std::make_shared<Registry>())
{
}

bool ActionHandler::handle_message(const json & request, json & response_out)
{
  std::string op = request["op"];
  if (op == "send_action_goal") {
    return send_goal(request, response_out);
  }
  if (op == "cancel_action_goal") {
    return cancel_goal(request, response_out);
  }
  return false;
}

std::string ActionHandler::resolve_action_type(const std::string & action)
{
  auto topics = node_->get_topic_names_and_types();
  auto it = topics.find(action + "/_action/feedback");
  if (it == topics.end() || it->second.empty()) {
    return "";
  }
  const std::string & feedback_type = it->second[0];
  auto pos = feedback_type.rfind(kFeedbackSuffix);
  if (pos == std::string::npos) {
    return "";
  }
  return feedback_type.substr(0, pos);
}

GenericClient::SharedPtr ActionHandler::get_client(
  const std::string & service, const std::string & type)
{
  auto it = clients_.find(service);
  if (it == clients_.end()) {
    it = clients_
           .emplace(
             service,
             node_->create_generic_client(service, type, rmw_qos_profile_services_default, nullptr))
           .first;
  }
  return it->second;
}

bool ActionHandler::send_goal(const json & request, json & response_out)
{
  std::string action = request.value("action", "");
  json id = request.value("id", json());
  std::string goal_key = id.dump();
  response_out["op"] = "send_action_goal";

  std::string action_type = normalize_action_type(request.value("action_type", ""));
  if (action_type.empty()) {
    action_type = resolve_action_type(action);
  }
  if (action.empty() || action_type.empty()) {
    response_out = make_result_error(id, action, "Action " + action + " not available");
    RCLCPP_ERROR(logger(), "%s", response_out["error"].get<std::string>().c_str());
    return true;
  }

  try {
    auto send_goal_client = get_client(action + "/_action/send_goal", action_type + "_SendGoal");
    auto get_result_client = get_client(action + "/_action/get_result", action_type + "_GetResult");
    if (!send_goal_client->service_is_ready() && !send_goal_client->wait_for_service(1s)) {
      response_out = make_result_error(id, action, "Action server " + action + " not ready");
      RCLCPP_ERROR(logger(), "%s", response_out["error"].get<std::string>().c_str());
      return true;
    }

    json uuid = random_uuid();
    std::string feedback_type = action_type + "_FeedbackMessage";

    auto goal = std::make_shared<ActiveGoal>();
    goal->action = action;
    goal->uuid = uuid;
    // Subscribe before sending the goal so early feedback isn't missed.
    goal->feedback_sub = node_->create_generic_subscription(
      action + "/_action/feedback", feedback_type, rclcpp::QoS(10),
      [send = send_, id, action, uuid,
       feedback_type](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
        try {
          json j = serialized_message_to_json(feedback_type, msg);
          if (j.value("goal_id", json::object()).value("uuid", json()) != uuid) {
            return;
          }
          json m = {
            {"id", id}, {"op", "action_feedback"}, {"action", action},
            {"values", j.value("feedback", json::object())}};
          send_json(send, std::move(m));
        } catch (const std::exception & e) {
          RCLCPP_ERROR(logger(), "Failed to translate action feedback: %s", e.what());
        }
      });

    {
      std::lock_guard<std::mutex> lock(registry_->mutex);
      registry_->goals[goal_key] = goal;
    }

    auto finish_goal = [registry = registry_, goal_key]() {
      std::lock_guard<std::mutex> lock(registry->mutex);
      registry->goals.erase(goal_key);  // drops the feedback subscription
    };

    json goal_req = {
      {"goal_id", {{"uuid", uuid}}},
      {"goal", request.contains("args") ? request["args"] : json::object()}};
    auto serialized_goal =
      json_to_serialized_service_request(action_type + "_SendGoal", goal_req);

    send_goal_client->async_send_request(
      serialized_goal,
      [send = send_, id, action, action_type, uuid, get_result_client,
       finish_goal](GenericClient::SharedFuture future) {
        try {
          json res = serialized_service_response_to_json(action_type + "_SendGoal", future.get());
          if (!res.value("accepted", false)) {
            finish_goal();
            send_json(send, make_result_error(id, action, "Goal rejected by action server"));
            return;
          }

          // Resolves once the goal reaches a terminal state (succeeded,
          // canceled or aborted) — that response is the action result.
          json result_req = {{"goal_id", {{"uuid", uuid}}}};
          auto serialized_result_req =
            json_to_serialized_service_request(action_type + "_GetResult", result_req);
          get_result_client->async_send_request(
            serialized_result_req,
            [send, id, action, action_type, finish_goal](GenericClient::SharedFuture result_future) {
              json m = {{"id", id}, {"op", "action_result"}, {"action", action}};
              try {
                json res = serialized_service_response_to_json(
                  action_type + "_GetResult", result_future.get());
                m["values"] = res.value("result", json::object());
                m["status"] = res.value("status", 0);
                m["result"] = true;
              } catch (const std::exception & e) {
                m["values"] = json::object();
                m["status"] = 0;
                m["result"] = false;
                m["error"] = std::string("Failed to translate action result: ") + e.what();
                RCLCPP_ERROR(logger(), "%s", m["error"].get<std::string>().c_str());
              }
              finish_goal();
              send_json(send, std::move(m));
            });
        } catch (const std::exception & e) {
          finish_goal();
          send_json(
            send, make_result_error(id, action, std::string("Failed to send goal: ") + e.what()));
          RCLCPP_ERROR(logger(), "Failed to send goal on %s: %s", action.c_str(), e.what());
        }
      });
  } catch (const std::exception & e) {
    {
      std::lock_guard<std::mutex> lock(registry_->mutex);
      registry_->goals.erase(goal_key);
    }
    response_out = make_result_error(id, action, std::string("Failed to call action: ") + e.what());
    RCLCPP_ERROR(logger(), "%s", response_out["error"].get<std::string>().c_str());
    return true;
  }

  // Ack; feedback and the final result arrive asynchronously under the same id.
  response_out["result"] = true;
  return true;
}

bool ActionHandler::cancel_goal(const json & request, json & response_out)
{
  json id = request.value("id", json());
  response_out["op"] = "cancel_action_goal";

  std::shared_ptr<ActiveGoal> goal;
  {
    std::lock_guard<std::mutex> lock(registry_->mutex);
    auto it = registry_->goals.find(id.dump());
    if (it != registry_->goals.end()) {
      goal = it->second;
    }
  }
  if (!goal) {
    response_out["error"] = "No active goal with id " + id.dump();
    RCLCPP_WARN(logger(), "%s", response_out["error"].get<std::string>().c_str());
    return true;
  }

  try {
    auto client = get_client(goal->action + "/_action/cancel_goal", kCancelGoalSrv);
    if (!client->service_is_ready() && !client->wait_for_service(1s)) {
      response_out["error"] = "Cancel service for " + goal->action + " not ready";
      RCLCPP_ERROR(logger(), "%s", response_out["error"].get<std::string>().c_str());
      return true;
    }
    // stamp omitted -> zero, which cancels exactly this goal id.
    json cancel_req = {{"goal_info", {{"goal_id", {{"uuid", goal->uuid}}}}}};
    auto serialized = json_to_serialized_service_request(kCancelGoalSrv, cancel_req);
    client->async_send_request(
      serialized,
      [send = send_, id, action = goal->action](GenericClient::SharedFuture future) {
        // The terminal action_result (status CANCELED) still arrives via the
        // pending get_result request; this only reports the cancel outcome.
        json m = {{"id", id}, {"op", "cancel_action_goal"}, {"action", action}, {"result", true}};
        try {
          m["values"] = serialized_service_response_to_json(kCancelGoalSrv, future.get());
        } catch (const std::exception & e) {
          m["result"] = false;
          m["error"] = std::string("Failed to translate cancel response: ") + e.what();
          RCLCPP_ERROR(logger(), "%s", m["error"].get<std::string>().c_str());
        }
        send_json(send, std::move(m));
      });
  } catch (const std::exception & e) {
    response_out["error"] = std::string("Failed to cancel goal: ") + e.what();
    RCLCPP_ERROR(logger(), "%s", response_out["error"].get<std::string>().c_str());
    return true;
  }

  response_out["result"] = true;
  return true;
}

}  // namespace rws
