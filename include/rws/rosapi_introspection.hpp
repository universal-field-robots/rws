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

#ifndef RWS__ROSAPI_INTROSPECTION_HPP_
#define RWS__ROSAPI_INTROSPECTION_HPP_

#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rws/node_interface.hpp"

namespace rws
{

using json = nlohmann::ordered_json;

/// Build rosapi TypeDef JSON (root first, then dependency types) for a message
/// type, e.g. "std_msgs/msg/Header". Constants are not exposed by introspection
/// typesupport, so constnames/constvalues are always empty. fieldarraylen
/// follows rosapi semantics: -1 scalar, 0 variable-length array, N fixed array.
json typedefs_for_message(const std::string & msg_type, bool rosbridge_compatible = false);

/// TypeDef JSON for the request (or response) message of a service type,
/// e.g. "std_srvs/srv/SetBool".
json typedefs_for_service(
  const std::string & srv_type, bool request, bool rosbridge_compatible = false);

/// Extract action server namespaces from a topic list: any <ns> having both
/// <ns>/_action/feedback and <ns>/_action/status. Returns sorted names.
std::vector<std::string> filter_action_servers(const std::vector<std::string> & topics);

/// Serves the remaining /rosapi/* introspection services natively so the
/// python rosapi node is not needed at all: services, topics_for_type,
/// action_servers, service_providers, service_node, get_time, get_ros_version,
/// message_details, service_request_details, service_response_details.
/// All are answered synchronously from the local graph cache / typesupport.
class RosapiIntrospection
{
public:
  RosapiIntrospection(std::shared_ptr<NodeInterface<>> node, bool rosbridge_compatible);

  /// Handle one rosbridge "call_service" request if it targets one of the
  /// services above. Returns false when it is not one of ours.
  bool handle_service_call(const json & request, json & response_out);

private:
  std::shared_ptr<NodeInterface<>> node_;
  bool rosbridge_compatible_;
};

}  // namespace rws

#endif  // RWS__ROSAPI_INTROSPECTION_HPP_
