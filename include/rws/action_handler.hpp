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

#ifndef RWS__ACTION_HANDLER_HPP_
#define RWS__ACTION_HANDLER_HPP_

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rws/generic_client.hpp"
#include "rws/node_interface.hpp"

namespace rws
{

using json = nlohmann::ordered_json;

/// Normalize "pkg/Name" to "pkg/action/Name"; passes "pkg/action/Name" through.
std::string normalize_action_type(const std::string & type);

/// ROS 2 action client support over the rosbridge protocol, built on the
/// action's underlying interfaces instead of rcl_action (which has no generic,
/// type-erased client): <action>/_action/send_goal and get_result services via
/// GenericClient, plus a generic subscription on <action>/_action/feedback.
///
/// Ops (upstream rosbridge naming, correlated by the client-chosen "id"):
///   -> {op:"send_action_goal", id, action, action_type, args, ...}
///   <- {op:"action_feedback", id, action, values}          (per feedback msg)
///   <- {op:"action_result", id, action, values, status, result}  (terminal)
///   -> {op:"cancel_action_goal", id, action}
///
/// A goal that fails up front (unknown type, server missing, rejection) still
/// produces an action_result with result:false so the GUI always resolves.
class ActionHandler
{
public:
  /// send_callback must stay valid independently of this object's lifetime
  /// (deferred feedback/result messages hold copies of it).
  ActionHandler(
    std::shared_ptr<NodeInterface<>> node, std::function<void(std::string & msg)> send_callback);

  /// Handle one message if its op is an action op. Returns false otherwise.
  bool handle_message(const json & request, json & response_out);

private:
  struct ActiveGoal
  {
    std::string action;
    json uuid;  // 16-byte array, the goal_id on the wire
    std::shared_ptr<rclcpp::GenericSubscription> feedback_sub;
  };
  struct Registry
  {
    std::mutex mutex;
    std::map<std::string, std::shared_ptr<ActiveGoal>> goals;  // key: dumped client id
  };

  bool send_goal(const json & request, json & response_out);
  bool cancel_goal(const json & request, json & response_out);

  GenericClient::SharedPtr get_client(const std::string & service, const std::string & type);
  /// Resolve the action type from the graph via the feedback topic's type.
  std::string resolve_action_type(const std::string & action);

  std::shared_ptr<NodeInterface<>> node_;
  std::function<void(std::string & msg)> send_;
  std::map<std::string, GenericClient::SharedPtr> clients_;
  // Shared with async callbacks so goal cleanup stays valid regardless of
  // handler lifetime.
  std::shared_ptr<Registry> registry_;
};

}  // namespace rws

#endif  // RWS__ACTION_HANDLER_HPP_
