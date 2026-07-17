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

#include "rws/rosapi_params.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rws/translate.hpp"

namespace rws
{

using namespace std::chrono_literals;

namespace
{

constexpr const char * kGetParametersSrv = "rcl_interfaces/srv/GetParameters";
constexpr const char * kSetParametersSrv = "rcl_interfaces/srv/SetParameters";
constexpr const char * kListParametersSrv = "rcl_interfaces/srv/ListParameters";

// rcl_interfaces/msg/ParameterValue field holding the value, indexed by ParameterType.
constexpr std::array<const char *, 10> kParamValueField = {
  "",
  "bool_value",
  "integer_value",
  "double_value",
  "string_value",
  "byte_array_value",
  "bool_array_value",
  "integer_array_value",
  "double_array_value",
  "string_array_value",
};

rclcpp::Logger logger() { return rclcpp::get_logger("rws_rosapi_params"); }

// rosapi values travel as JSON-encoded strings; like the python rosapi, a
// string that fails to parse as JSON is kept as a plain string value.
json parse_json_or_string(const std::string & s)
{
  json v = json::parse(s, nullptr, false);
  return v.is_discarded() ? json(s) : v;
}

json request_args(const json & request)
{
  return request.contains("args") && request["args"].is_object() ? request["args"]
                                                                 : json::object();
}

void send_json(const std::function<void(std::string & msg)> & send, json msg)
{
  std::string s = msg.dump();
  if (send) {
    send(s);
  }
}

json make_response(const json & id, const std::string & service)
{
  return {{"id", id}, {"op", "service_response"}, {"service", service}, {"result", true}};
}

void warn_malformed_name(const std::string & name)
{
  RCLCPP_WARN(
    logger(), "Malformed parameter name: %s; expecting <node_name>:<param_name>", name.c_str());
}

template <typename Pred>
bool all_elements(const json & arr, Pred pred)
{
  return std::all_of(arr.begin(), arr.end(), pred);
}

}  // namespace

std::optional<json> param_value_to_json(const json & param_value)
{
  size_t type = param_value.value("type", 0);
  if (
    type == 0 || type >= kParamValueField.size() ||
    !param_value.contains(kParamValueField[type])) {
    return std::nullopt;
  }
  return param_value[kParamValueField[type]];
}

json json_to_param_value(const json & value)
{
  if (value.is_boolean()) {
    return {{"type", 1}, {"bool_value", value}};
  }
  if (value.is_number_integer()) {
    return {{"type", 2}, {"integer_value", value}};
  }
  if (value.is_number_float()) {
    return {{"type", 3}, {"double_value", value}};
  }
  if (value.is_string()) {
    return {{"type", 4}, {"string_value", value}};
  }
  if (value.is_array()) {
    if (all_elements(value, [](const json & v) { return v.is_boolean(); })) {
      return {{"type", 6}, {"bool_array_value", value}};
    }
    if (all_elements(value, [](const json & v) { return v.is_number_integer(); })) {
      return {{"type", 7}, {"integer_array_value", value}};
    }
    if (all_elements(value, [](const json & v) { return v.is_number(); })) {
      return {{"type", 8}, {"double_array_value", value}};
    }
    if (all_elements(value, [](const json & v) { return v.is_string(); })) {
      return {{"type", 9}, {"string_array_value", value}};
    }
  }
  return {{"type", 4}, {"string_value", value.dump()}};
}

bool split_param_name(
  const std::string & name, std::string & node_name_out, std::string & param_name_out)
{
  auto pos = name.find(':');
  if (pos == std::string::npos || pos == 0 || pos + 1 >= name.size()) {
    return false;
  }
  node_name_out = name.substr(0, pos);
  param_name_out = name.substr(pos + 1);
  if (node_name_out[0] != '/') {
    node_name_out = "/" + node_name_out;
  }
  return true;
}

RosapiParams::RosapiParams(
  std::shared_ptr<NodeInterface<>> node, std::function<void(std::string & msg)> send_callback)
: node_(node), send_(send_callback)
{
}

bool RosapiParams::handle_service_call(const json & request, json & response_out)
{
  std::string service = request["service"];
  if (service == "/rosapi/get_param") {
    return get_param(request, response_out);
  }
  if (service == "/rosapi/set_param") {
    return set_param(request, response_out);
  }
  if (service == "/rosapi/has_param") {
    return has_param(request, response_out);
  }
  if (service == "/rosapi/delete_param") {
    return delete_param(request, response_out);
  }
  if (service == "/rosapi/get_param_names") {
    return get_param_names(request, response_out);
  }
  return false;
}

bool RosapiParams::get_param(const json & request, json & response_out)
{
  json args = request_args(request);
  std::string fallback = parse_json_or_string(args.value("default_value", "")).dump();

  std::string node_name, param_name;
  if (!split_param_name(args.value("name", ""), node_name, param_name)) {
    warn_malformed_name(args.value("name", ""));
    response_out["values"]["value"] = "";
    response_out["result"] = true;
    return true;
  }

  if (!node_exists(node_name)) {
    response_out["values"]["value"] = fallback;
    response_out["result"] = true;
    return true;
  }

  json id = request.value("id", json());
  try {
    auto client = get_ready_client(node_name + "/get_parameters", kGetParametersSrv);
    if (!client) {
      response_out["values"]["value"] = fallback;
      response_out["result"] = true;
      return true;
    }
    auto req = json_to_serialized_service_request(
      kGetParametersSrv, json{{"names", json::array({param_name})}});
    client->async_send_request(
      req, [send = send_, id, fallback](GenericClient::SharedFuture future) {
        json m = make_response(id, "/rosapi/get_param");
        std::string value = fallback;
        try {
          json res = serialized_service_response_to_json(kGetParametersSrv, future.get());
          if (res.contains("values") && !res["values"].empty()) {
            if (auto v = param_value_to_json(res["values"][0])) {
              value = v->dump();
            }
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(logger(), "Failed to translate get_parameters response: %s", e.what());
        }
        m["values"]["value"] = value;
        send_json(send, std::move(m));
      });
  } catch (const std::exception & e) {
    response_out["error"] = std::string("Failed to call get_parameters: ") + e.what();
    RCLCPP_ERROR(logger(), "%s", response_out["error"].get<std::string>().c_str());
    return true;
  }

  // Ack now; the real response is sent from the reply callback (same deferred
  // pattern as ClientHandler::call_external_service).
  response_out["op"] = "call_service";
  response_out["result"] = true;
  return true;
}

bool RosapiParams::set_param(const json & request, json & response_out)
{
  json args = request_args(request);

  std::string node_name, param_name;
  if (!split_param_name(args.value("name", ""), node_name, param_name)) {
    warn_malformed_name(args.value("name", ""));
    response_out["values"] = json::object();
    response_out["result"] = true;
    return true;
  }

  json value = json::parse(args.value("value", ""), nullptr, false);
  if (value.is_discarded()) {
    response_out["error"] = "set_param value must be a JSON-formatted string";
    RCLCPP_ERROR(logger(), "%s", response_out["error"].get<std::string>().c_str());
    return true;
  }

  if (!node_exists(node_name)) {
    // The python rosapi silently ignored sets on missing nodes; keep that
    // contract so the GUI sees the same (successful) response.
    RCLCPP_WARN(logger(), "set_param: node %s not found", node_name.c_str());
    response_out["values"] = json::object();
    response_out["result"] = true;
    return true;
  }

  return send_set_parameters(
    node_name, param_name, json_to_param_value(value), "/rosapi/set_param", request, response_out);
}

bool RosapiParams::has_param(const json & request, json & response_out)
{
  json args = request_args(request);

  std::string node_name, param_name;
  if (!split_param_name(args.value("name", ""), node_name, param_name)) {
    warn_malformed_name(args.value("name", ""));
    response_out["values"]["exists"] = false;
    response_out["result"] = true;
    return true;
  }

  if (!node_exists(node_name)) {
    response_out["values"]["exists"] = false;
    response_out["result"] = true;
    return true;
  }

  json id = request.value("id", json());
  try {
    auto client = get_ready_client(node_name + "/get_parameters", kGetParametersSrv);
    if (!client) {
      response_out["values"]["exists"] = false;
      response_out["result"] = true;
      return true;
    }
    auto req = json_to_serialized_service_request(
      kGetParametersSrv, json{{"names", json::array({param_name})}});
    client->async_send_request(req, [send = send_, id](GenericClient::SharedFuture future) {
      json m = make_response(id, "/rosapi/has_param");
      bool exists = false;
      try {
        json res = serialized_service_response_to_json(kGetParametersSrv, future.get());
        if (res.contains("values") && !res["values"].empty()) {
          exists = param_value_to_json(res["values"][0]).has_value();
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(logger(), "Failed to translate get_parameters response: %s", e.what());
      }
      m["values"]["exists"] = exists;
      send_json(send, std::move(m));
    });
  } catch (const std::exception & e) {
    response_out["error"] = std::string("Failed to call get_parameters: ") + e.what();
    RCLCPP_ERROR(logger(), "%s", response_out["error"].get<std::string>().c_str());
    return true;
  }

  response_out["op"] = "call_service";
  response_out["result"] = true;
  return true;
}

bool RosapiParams::delete_param(const json & request, json & response_out)
{
  json args = request_args(request);

  std::string node_name, param_name;
  if (!split_param_name(args.value("name", ""), node_name, param_name)) {
    warn_malformed_name(args.value("name", ""));
    response_out["values"] = json::object();
    response_out["result"] = true;
    return true;
  }

  if (!node_exists(node_name)) {
    response_out["values"] = json::object();
    response_out["result"] = true;
    return true;
  }

  // Deleting is setting to PARAMETER_NOT_SET (type 0), as the python rosapi did.
  return send_set_parameters(
    node_name, param_name, json{{"type", 0}}, "/rosapi/delete_param", request, response_out);
}

bool RosapiParams::send_set_parameters(
  const std::string & node_name, const std::string & param_name, const json & param_value,
  const std::string & rosapi_service, const json & request, json & response_out)
{
  json id = request.value("id", json());
  try {
    auto client = get_ready_client(node_name + "/set_parameters", kSetParametersSrv);
    if (!client) {
      RCLCPP_WARN(
        logger(), "%s: %s/set_parameters not ready", rosapi_service.c_str(), node_name.c_str());
      response_out["values"] = json::object();
      response_out["result"] = true;
      return true;
    }
    auto req = json_to_serialized_service_request(
      kSetParametersSrv,
      json{{"parameters", json::array({json{{"name", param_name}, {"value", param_value}}})}});
    client->async_send_request(
      req, [send = send_, id, rosapi_service, node_name,
            param_name](GenericClient::SharedFuture future) {
        // The rosapi set/delete responses carry no fields; failures are only
        // logged, matching the python rosapi's silent behaviour.
        json m = make_response(id, rosapi_service);
        m["values"] = json::object();
        try {
          json res = serialized_service_response_to_json(kSetParametersSrv, future.get());
          if (
            res.contains("results") && !res["results"].empty() &&
            !res["results"][0].value("successful", false)) {
            RCLCPP_WARN(
              logger(), "%s: setting %s on %s failed: %s", rosapi_service.c_str(),
              param_name.c_str(), node_name.c_str(),
              res["results"][0].value("reason", "").c_str());
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(logger(), "Failed to translate set_parameters response: %s", e.what());
        }
        send_json(send, std::move(m));
      });
  } catch (const std::exception & e) {
    response_out["error"] = std::string("Failed to call set_parameters: ") + e.what();
    RCLCPP_ERROR(logger(), "%s", response_out["error"].get<std::string>().c_str());
    return true;
  }

  response_out["op"] = "call_service";
  response_out["result"] = true;
  return true;
}

bool RosapiParams::get_param_names(const json & request, json & response_out)
{
  json id = request.value("id", json());

  std::vector<std::pair<std::string, GenericClient::SharedPtr>> clients;
  for (const auto & node_name : node_->get_node_names()) {
    try {
      auto client = get_client(node_name + "/list_parameters", kListParametersSrv);
      // Non-blocking readiness check: a node whose parameter service hasn't
      // been discovered yet is skipped rather than waited on, so one absent
      // node can't stall the whole listing.
      if (client->service_is_ready()) {
        clients.emplace_back(node_name, client);
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger(), "get_param_names: skipping %s: %s", node_name.c_str(), e.what());
    }
  }

  if (clients.empty()) {
    response_out["values"]["names"] = json::array();
    response_out["result"] = true;
    return true;
  }

  struct Gather
  {
    std::mutex mutex;
    std::vector<std::string> names;
    size_t pending;
    bool responded = false;
  };
  auto state = std::make_shared<Gather>();
  state->pending = clients.size();

  auto respond = [send = send_, id](std::vector<std::string> names) {
    json m = make_response(id, "/rosapi/get_param_names");
    m["values"]["names"] = names;
    send_json(send, std::move(m));
  };

  auto finish_one = [state, respond](std::vector<std::string> new_names) {
    std::vector<std::string> snapshot;
    bool do_respond = false;
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->names.insert(state->names.end(), new_names.begin(), new_names.end());
      if (--state->pending == 0 && !state->responded) {
        state->responded = true;
        do_respond = true;
        snapshot = state->names;
      }
    }
    if (do_respond) {
      respond(std::move(snapshot));
    }
  };

  auto req = json_to_serialized_service_request(
    kListParametersSrv, json{{"prefixes", json::array()}, {"depth", 0}});

  for (const auto & entry : clients) {
    const std::string node_name = entry.first;
    try {
      entry.second->async_send_request(
        req, [node_name, finish_one](GenericClient::SharedFuture future) {
          std::vector<std::string> names;
          try {
            json res = serialized_service_response_to_json(kListParametersSrv, future.get());
            for (const auto & n : res["result"]["names"]) {
              names.push_back(node_name + ":" + n.get<std::string>());
            }
          } catch (const std::exception & e) {
            RCLCPP_WARN(
              logger(), "get_param_names: bad response from %s: %s", node_name.c_str(), e.what());
          }
          finish_one(std::move(names));
        });
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        logger(), "get_param_names: request to %s failed: %s", node_name.c_str(), e.what());
      finish_one({});
    }
  }

  // Watchdog: one node that accepted the request but never replies must not
  // hang the GUI's request forever; after the deadline whatever has been
  // gathered is sent. Captures are all by value, so this outliving the client
  // handler is safe.
  std::thread([state, respond]() {
    std::this_thread::sleep_for(2s);
    std::vector<std::string> snapshot;
    bool do_respond = false;
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (!state->responded) {
        state->responded = true;
        do_respond = true;
        snapshot = state->names;
      }
    }
    if (do_respond) {
      respond(std::move(snapshot));
    }
  }).detach();

  response_out["op"] = "call_service";
  response_out["result"] = true;
  return true;
}

GenericClient::SharedPtr RosapiParams::get_client(
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

GenericClient::SharedPtr RosapiParams::get_ready_client(
  const std::string & service, const std::string & type)
{
  auto client = get_client(service, type);
  // The target node is known to be on the graph, so its parameter services are
  // normally discovered already; the wait only covers the brief discovery race
  // and is bounded so it can't stall message processing the way the python
  // rosapi's 5s serialized wait did.
  if (!client->service_is_ready() && !client->wait_for_service(1s)) {
    return nullptr;
  }
  return client;
}

bool RosapiParams::node_exists(const std::string & node_name)
{
  auto names = node_->get_node_names();
  return std::find(names.begin(), names.end(), node_name) != names.end();
}

}  // namespace rws
