// Copyright 2022 Vasily Kiniv
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

#include "rws/client_handler.hpp"

#include <chrono>
#include <cstdio>
#include <nlohmann/json.hpp>

#include "rclcpp/logger.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rws/translate.hpp"

namespace rws
{

using json = nlohmann::ordered_json;
using namespace std::chrono_literals;
using std::placeholders::_1;

std::string string_thread_id()
{
  auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
  return std::to_string(hashed);
}

ClientHandler::ClientHandler(
  int client_id, std::shared_ptr<rws::NodeInterface<>> node, std::shared_ptr<Connector<>> connector,
  bool rosbridge_compatible, std::function<void(std::string & msg)> callback,
  std::function<void(std::vector<std::uint8_t> & msg)> binary_callback)
: client_id_(client_id),
  node_(node),
  connector_(connector),
  rosbridge_compatible_(rosbridge_compatible),
  callback_(callback),
  binary_callback_(binary_callback),
  // Deferred param responses must outlive this handler, so RosapiParams gets
  // the raw transport callback rather than a wrapper around this object.
  rosapi_params_(node, callback),
  rosapi_introspection_(node, rosbridge_compatible),
  action_handler_(node, callback)
{
  RCLCPP_INFO(
    get_logger(), "Constructing client %s(%s)", std::to_string(client_id_).c_str(),
    string_thread_id().c_str());
}

ClientHandler::~ClientHandler()
{
  RCLCPP_INFO(
    get_logger(), "Destroying client %s(%s)", std::to_string(client_id_).c_str(),
    string_thread_id().c_str());
  for (auto it = subscriptions_.begin(); it != subscriptions_.end(); ++it) {
    it->second();
  }
  for (auto it = publishers_.begin(); it != publishers_.end(); ++it) {
    it->second();
  }
}

json ClientHandler::process_message(json & msg)
{
  bool handled = false;
  json response = {{"id", msg["id"]}, {"result", false}};

  if (!msg.contains("op")) {
    response["error"] = "No op specified";
    RCLCPP_ERROR(get_logger(), response["error"].dump().c_str());
    return response;
  }

  std::string op = msg["op"];

  if (op == "call_service") {
    handled = call_service(msg, response);
  }

  if (op == "subscribe") {
    handled = subscribe_to_topic(msg, response);
  }

  if (op == "advertise") {
    handled = advertise_topic(msg, response);
  }

  if (op == "unadvertise") {
    handled = unadvertise_topic(msg, response);
  }

  if (op == "publish") {
    handled = publish_to_topic(msg, response);
  }

  if (op == "unsubscribe") {
    handled = unsubscribe_from_topic(msg, response);
  }

  if (op == "send_action_goal" || op == "cancel_action_goal") {
    handled = action_handler_.handle_message(msg, response);
  }

  if (!handled) {
    RCLCPP_WARN(get_logger(), "Unhadled request: %s", msg.dump().c_str());
  }

  return response;
}

void ClientHandler::send_message(std::string & msg)
{
  if (this->callback_) {
    this->callback_(msg);
  }
}

void ClientHandler::send_message(std::vector<std::uint8_t> & msg)
{
  if (this->binary_callback_) {
    this->binary_callback_(msg);
  }
}

void ClientHandler::subscription_callback(topic_params & params, std::shared_ptr<const rclcpp::SerializedMessage> message)
{
  uint32_t secs = node_->now().seconds();
  uint32_t nsecs = node_->now().nanoseconds() - (secs * 1000000000);

  json m = {
    {"op", "publish"},
    {"topic", params.topic},
  };

  auto compression = params.compression;
  auto sub_type = params.type;

  if (compression == "cbor-raw") {
    auto buf = std::vector<std::uint8_t>(
      &message->get_rcl_serialized_message().buffer[0],
      &message->get_rcl_serialized_message()
          .buffer[message->get_rcl_serialized_message().buffer_length]);
    m["msg"] = {{"secs", secs}, {"nsecs", nsecs}, {"bytes", json::binary_t(buf)}};
    std::vector<std::uint8_t> cbor_buf = json::to_cbor(m);
    this->send_message(cbor_buf);
  } else if (compression == "cbor") {
    m["msg"] = rws::serialized_message_to_json(sub_type, std::move(message));
    std::vector<std::uint8_t> buf = json::to_cbor(m);
    this->send_message(buf);
  } else if (compression == "bson") {
    m["msg"] = rws::serialized_message_to_json(sub_type, message);
    std::vector<std::uint8_t> buf = json::to_bson(m);
    this->send_message(buf);
  } else if (compression == "msgpack") {
    m["msg"] = rws::serialized_message_to_json(sub_type, message);
    std::vector<std::uint8_t> buf = json::to_msgpack(m);
    this->send_message(buf);
  } else if (compression == "ubjson") {
    m["msg"] = rws::serialized_message_to_json(sub_type, message);
    std::vector<std::uint8_t> buf = json::to_ubjson(m);
    this->send_message(buf);
  } else if (compression == "bjdata") {
    m["msg"] = rws::serialized_message_to_json(sub_type, message);
    std::vector<std::uint8_t> buf = json::to_bjdata(m);
    this->send_message(buf);
  } else {
    m["msg"] = rws::serialized_message_to_json(sub_type, message);
    std::string json_str = m.dump();
    this->send_message(json_str);
  }
}

bool ClientHandler::subscribe_to_topic(const json & msg, json & response)
{
  response["op"] = "subscribe_response";
  if (!msg.contains("topic") || !msg["topic"].is_string()) {
    response["result"] = false;
    response["error"] = "No topic specified";
    RCLCPP_ERROR(get_logger(), response["error"].dump().c_str());
    return true;
  }

  std::string topic = msg["topic"];
  std::map<std::string, std::vector<std::string>> topics = node_->get_topic_names_and_types();

  std::string sub_type;
  auto topic_it = topics.find(topic);
  if (topic_it != topics.end() && !topic_it->second.empty()) {
    // Type discovered from the live ROS graph.
    sub_type = topic_it->second[0];
  } else if (msg.contains("type") && msg["type"].is_string()) {
    // Topic has no publisher yet. Rosbridge clients may subscribe ahead of the
    // publisher by supplying the message type; honour it so the subscription
    // starts delivering once a publisher appears. This also stops clients that
    // periodically retry the subscription from spamming the log with errors.
    sub_type = rws::message_type_to_ros2_style(msg["type"]);
    RCLCPP_INFO(
      get_logger(), "Topic '%s' not yet advertised; subscribing with provided type '%s'",
      topic.c_str(), sub_type.c_str());
  } else {
    static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
    response["error"] = "Topic " + topic + " not found and no type provided";
    response["result"] = false;
    RCLCPP_ERROR_THROTTLE(
      get_logger(), throttle_clock, 5000, "Failed to subscribe to topic: %s",
      response["error"].dump().c_str());
    return true;
  }

  size_t history_depth = 10;
  if (msg.contains("history_depth") && msg["history_depth"].is_number()) {
    history_depth = msg["history_depth"];
  } else if (msg.contains("queue_size") && msg["queue_size"].is_number()) {
    history_depth = msg["queue_size"];
  }
  rclcpp::Duration throttle_rate(0, 0);
  if (msg.contains("throttle_rate") && msg["throttle_rate"].is_number()) {
    size_t throttle_rate_ms = msg["throttle_rate"];
    throttle_rate = rclcpp::Duration(0, throttle_rate_ms * 1000000);
  }
  std::string compression =
    (!msg.contains("compression") || !msg["compression"].is_string()) ? "none" : msg["compression"];

  if (subscriptions_.count(topic) == 0) {
    topic_params params(topic, sub_type, history_depth, compression, throttle_rate);
    subscriptions_[topic] = connector_->subscribe_to_topic(
      client_id_, params, std::bind(&ClientHandler::subscription_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

  // Always acknowledge, even if the subscription already existed, so the client
  // isn't left waiting for a response on a repeated subscribe.
  response["type"] = sub_type;
  response["result"] = true;

  return true;
}

bool ClientHandler::unsubscribe_from_topic(const json & msg, json & response)
{
  response["op"] = "unsubscribe_response";

  std::string topic = msg["topic"];
  if (subscriptions_.count(topic) > 0) {
    subscriptions_[topic]();
    subscriptions_.erase(topic);
    response["result"] = true;
  }

  return true;
}

bool ClientHandler::advertise_topic(const json & msg, json & response)
{
  response["op"] = "advertise_response";
  if (!msg.contains("type") || !msg["type"].is_string()) {
    response["result"] = false;
    response["error"] = "No type specified";
    RCLCPP_ERROR(get_logger(), response["error"].dump().c_str());
    return true;
  }

  if (!msg.contains("topic") || !msg["topic"].is_string()) {
    response["result"] = false;
    response["error"] = "No topic specified";
    RCLCPP_ERROR(get_logger(), response["error"].dump().c_str());
    return true;
  }

  std::string topic = msg["topic"];
  std::string type = rws::message_type_to_ros2_style(msg["type"]);
  size_t history_depth = 10;
  if (msg.contains("history_depth") && msg["history_depth"].is_number()) {
    history_depth = msg["history_depth"];
  } else if (msg.contains("queue_size") && msg["queue_size"].is_number()) {
    history_depth = msg["queue_size"];
  }
  bool latch =
    (msg.contains("latch") && msg["latch"].is_boolean()) ? msg["latch"].get<bool>() : false;
  topic_params params(topic, type, history_depth, latch);

  if (publishers_.count(topic) == 0) {
    publishers_[topic] = connector_->advertise_topic(client_id_, params, publisher_cb_[topic]);
    publisher_type_[topic] = type;
    response["result"] = true;
  }

  return true;
}

bool ClientHandler::unadvertise_topic(const json & msg, json & response)
{
  response["op"] = "unadvertise_response";

  std::string topic = msg["topic"];
  if (publishers_.count(topic) > 0) {
    publishers_[topic]();
    publishers_.erase(topic);
    publisher_cb_.erase(topic);
    publisher_type_.erase(topic);
    response["result"] = true;
  }

  return true;
}

bool ClientHandler::publish_to_topic(const json & msg, json & response)
{
  response["op"] = "publish_response";

  if (!msg.contains("topic")) {
    response["false"] = true;
    response["error"] = "No topic specified";
    RCLCPP_ERROR(get_logger(), response["error"].dump().c_str());
    return true;
  }

  std::string topic = msg["topic"];

  if (publishers_.count(topic) > 0) {
    json msg_json = msg["msg"];
    std::string type = publisher_type_[topic];
    auto serialized_msg = rws::json_to_serialized_message(type, msg_json);
    publisher_cb_[topic](serialized_msg);
    response["result"] = true;
  } else {
    response["result"] = false;
    response["error"] = "Topic was not advertised";
    RCLCPP_ERROR(get_logger(), response["error"].dump().c_str());
  }

  return true;
}

bool ClientHandler::call_service(const json & msg, json & response)
{
  if (!msg.contains("service")) {
    RCLCPP_ERROR(get_logger(), "No service specified");
    return true;
  }
  std::string service = msg["service"];

  response["op"] = "service_response";
  response["service"] = service;
  response["result"] = false;

  if (service == "/rosapi/topics_and_raw_types" || service == "/rosapi/topics") {
    response["values"]["topics"] = json::array();
    response["values"]["types"] = json::array();

    std::map<std::string, std::vector<std::string>> topics = node_->get_topic_names_and_types();
    for (auto it = topics.begin(); it != topics.end(); ++it) {
      response["values"]["topics"].push_back(it->first);
      response["values"]["types"].push_back(it->second[0]);

      if (msg["service"] == "/rosapi/topics_and_raw_types") {
        response["values"]["typedefs_full_text"].push_back(
          rws::generate_message_meta(it->second[0], rosbridge_compatible_));
      }
    }
    response["result"] = true;
    return true;
  } else if (service == "/rosapi/service_type") {
    std::string service_name = msg["args"]["service"];
    std::map<std::string, std::vector<std::string>> services = node_->get_service_names_and_types();
    if (services.find(service_name) == services.end()) {
      RCLCPP_ERROR(get_logger(), "Service not found: %s", service_name.c_str());
      return true;
    }

    std::string service_type = services[service_name][0];
    response["values"]["type"] = service_type;
    response["result"] = true;
    return true;
  } else if (service == "/rosapi/nodes") {
    response["values"]["nodes"] = json::array();

    std::vector<std::string> nodes = node_->get_node_names();
    for (auto it = nodes.begin(); it != nodes.end(); ++it) {
      response["values"]["nodes"].push_back(*it);
    }

    response["result"] = true;
    return true;
  }

  if (service == "/rosapi/publishers") {
    response["values"]["publishers"] = json::array();

    std::vector<rclcpp::TopicEndpointInfo> publishers = node_->get_publishers_info_by_topic(msg["args"]["topic"]);
    for (const auto & pub_info : publishers) {
      response["values"]["publishers"].push_back(("/" + pub_info.node_name()).c_str());
    }

    response["result"] = true;
    return true;
  }

  if (service == "/rosapi/subscribers") {
    response["values"]["subscribers"] = json::array();

    std::vector<rclcpp::TopicEndpointInfo> subscribers = node_->get_subscriptions_info_by_topic(msg["args"]["topic"]);
    for (const auto & sub_info : subscribers) {
      response["values"]["subscribers"].push_back(("/" + sub_info.node_name()).c_str());
    }

    response["result"] = true;
    return true;
  }

  if (service == "/rosapi/node_details") {
    response["values"]["subscribing"] = json::array();
    response["values"]["publishing"] = json::array();
    response["values"]["services"] = json::array();

    auto [ns, node_name] = split_ns_node_name(msg["args"]["node"]);

    std::map<std::string, std::vector<std::string>> topics = node_->get_topic_names_and_types();
    for (auto it = topics.begin(); it != topics.end(); ++it) {
      auto subscribers = node_->get_subscriptions_info_by_topic(it->first);
      for (auto sub_it = subscribers.begin(); sub_it != subscribers.end(); ++sub_it) {
        std::string sub_node = sub_it->node_name();
        std::string sub_ns = sub_it->node_namespace() == "/" ? "" : sub_it->node_namespace();
        if (sub_ns + sub_node == ns + node_name) {
          response["values"]["subscribing"].push_back(it->first);
        }
      }

      auto publishers = node_->get_publishers_info_by_topic(it->first);
      for (auto pub_it = publishers.begin(); pub_it != publishers.end(); ++pub_it) {
        std::string pub_node = pub_it->node_name();
        std::string pub_ns = pub_it->node_namespace() == "/" ? "" : pub_it->node_namespace();
        if (pub_ns + pub_node == ns + node_name) {
          response["values"]["publishing"].push_back(it->first);
        }
      }
    }

    try {
      auto services = node_->get_service_names_and_types_by_node(node_name, ns);
      for (auto it = services.begin(); it != services.end(); ++it) {
        response["values"]["services"].push_back(it->first);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_logger(), "Exception while fetching services for node(%s), ns=%s, name=%s: %s",
        msg["args"]["node"].get<std::string>().c_str(), ns.c_str(), node_name.c_str(), e.what());
    }

    response["result"] = true;
    return true;
  } else if (service == "/rosapi/topic_type") {
    std::string topic_name = msg["args"]["topic"].get<std::string>();
    std::map<std::string, std::vector<std::string>> topics = node_->get_topic_names_and_types(); 
    if (topics.find(topic_name) == topics.end()) {
      RCLCPP_ERROR(get_logger(), "Topic not found: %s", topic_name.c_str());
      return true;
    }

    response["values"]["type"] = topics[topic_name][0];
    response["result"] = true;
    return true;
  } else if (service == "/rosapi/services_for_type") {
    std::string service_type = msg["args"]["type"].get<std::string>();
    auto service_name_and_types = node_->get_service_names_and_types();

    std::map<std::string, std::vector<std::string>> filtered_service_name_and_types;
    for (const auto &pair : service_name_and_types) {
      for (const auto &type : pair.second) {
        if (type == service_type) {
          filtered_service_name_and_types.insert(pair);
          break;
        }
      }
    }

    response["values"]["services"] = json::array();
    for (auto it = filtered_service_name_and_types.begin(); it != filtered_service_name_and_types.end(); ++it) {
      response["values"]["services"].push_back(it->first);
    }

    response["result"] = true;
    return true;
  }

  if (rosapi_params_.handle_service_call(msg, response)) {
    return true;
  }

  if (rosapi_introspection_.handle_service_call(msg, response)) {
    return true;
  }

  return call_external_service(msg, response);
}

bool ClientHandler::call_external_service(const json & msg, json & response)
{
  std::string service_name = msg["service"];

  response["op"] = "service_response";
  response["service"] = service_name;
  response["result"] = false;

  // A generic client must be built from the service type that is actually
  // advertised on the ROS graph (e.g. "automine_msgs/srv/GetRobotDescription").
  // The type a rosbridge client sends (if any) is a ROS1-style hint (e.g.
  // "automine_msgs/GetRobotDescription") whose package layout doesn't match the
  // ROS2 typesupport library, so it can't be used to look up typesupport.
  // Resolving from the graph is therefore both more correct and the only thing
  // that reliably works.
  std::map<std::string, std::vector<std::string>> services = node_->get_service_names_and_types();
  auto svc_it = services.find(service_name);
  if (svc_it == services.end() || svc_it->second.empty()) {
    response["error"] = "Service " + service_name + " not available";
    RCLCPP_ERROR(get_logger(), "%s", response["error"].get<std::string>().c_str());
    // Return handled so the (failed) response is sent back; otherwise the client
    // waits forever for a reply that never comes.
    return true;
  }
  std::string service_type = svc_it->second[0];

  try {
    if (clients_.count(service_name) == 0) {
      clients_[service_name] = node_->create_generic_client(
        service_name, service_type, rmw_qos_profile_services_default, nullptr);
    }

    // The service is on the graph, so it should become ready almost immediately.
    // Bound the wait so a momentarily-unavailable service can't block the single
    // message-processing thread (and therefore every other client) indefinitely.
    if (!clients_[service_name]->wait_for_service(5s)) {
      response["error"] = "Service " + service_name + " did not become ready";
      RCLCPP_ERROR(get_logger(), "%s", response["error"].get<std::string>().c_str());
      return true;
    }

    const json & args = msg.contains("args") ? msg["args"] : json::object();
    auto serialized_req = json_to_serialized_service_request(service_type, args);

    using ServiceResponseFuture = rws::GenericClient::SharedFuture;
    auto response_received_callback = [this, id = msg.value("id", json()), service_name,
                                       service_type](ServiceResponseFuture future) {
      // This runs on an executor thread; an uncaught exception here would tear
      // down the executor, so translate failures into an error response instead.
      json m = {
        {"id", id},
        {"op", "service_response"},
        {"service", service_name},
      };
      try {
        m["values"] = serialized_service_response_to_json(service_type, future.get());
        m["result"] = true;
      } catch (const std::exception & e) {
        m["result"] = false;
        m["error"] = std::string("Failed to deserialize service response: ") + e.what();
        RCLCPP_ERROR(get_logger(), "%s", m["error"].get<std::string>().c_str());
      }
      std::string json_str = m.dump();
      this->send_message(json_str);
    };
    clients_[service_name]->async_send_request(serialized_req, response_received_callback);
  } catch (const std::exception & e) {
    // Drop a possibly half-initialised client so a later call can rebuild it.
    clients_.erase(service_name);
    response["error"] = std::string("Failed to call service ") + service_name + ": " + e.what();
    RCLCPP_ERROR(get_logger(), "%s", response["error"].get<std::string>().c_str());
    return true;
  }

  // Request dispatched successfully. Acknowledge synchronously; the real result
  // is delivered asynchronously from response_received_callback once the service
  // replies. (op "call_service" so the client doesn't mistake this ack for the
  // actual "service_response".)
  response["op"] = "call_service";
  response["result"] = true;
  return true;
}

}  // namespace rws
