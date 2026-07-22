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

#ifndef RWS__ROSAPI_PARAMS_HPP_
#define RWS__ROSAPI_PARAMS_HPP_

#include <functional>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rws/generic_client.hpp"
#include "rws/node_interface.hpp"

namespace rws
{

using json = nlohmann::ordered_json;

/// Extract the value held by a json-encoded rcl_interfaces/msg/ParameterValue.
/// Returns std::nullopt for PARAMETER_NOT_SET or an unknown type.
std::optional<json> param_value_to_json(const json & param_value);

/// Build a json-encoded rcl_interfaces/msg/ParameterValue from a plain json value.
/// Values with no ParameterValue slot (objects, null, mixed arrays) are stored
/// as their string representation, like the python rosapi did.
json json_to_param_value(const json & value);

/// Split rosapi's "<node_name>:<param_name>" convention on the first colon.
/// The node name is made absolute. Returns false when either part is empty.
bool split_param_name(
  const std::string & name, std::string & node_name_out, std::string & param_name_out);

/// Native replacement for the python rosapi node's parameter services
/// (/rosapi/get_param, set_param, has_param, delete_param, get_param_names).
///
/// The python implementation serialized every call behind a global lock while
/// doing a blocking 5s wait_for_service, so one lookup of a missing node/param
/// stalled every other GUI param call. Here each request is dispatched through
/// an async generic client to the target node's parameter services
/// (<node>/get_parameters etc.) and answered from the reply callback, so calls
/// never queue behind each other; requests for nodes that are not on the graph
/// are answered immediately with the default.
class RosapiParams
{
public:
  /// send_callback must stay valid independently of this object's lifetime:
  /// deferred responses (and the get_param_names watchdog) hold copies of it
  /// that can fire after the owning client handler is destroyed.
  RosapiParams(
    std::shared_ptr<NodeInterface<>> node, std::function<void(std::string & msg)> send_callback);

  /// Handle one rosbridge "call_service" request if it targets a param service.
  /// Returns false when the service is not one of ours so the caller can fall
  /// through to the generic external-service path.
  bool handle_service_call(const json & request, json & response_out);

private:
  bool get_param(const json & request, json & response_out);
  bool set_param(const json & request, json & response_out);
  bool has_param(const json & request, json & response_out);
  bool delete_param(const json & request, json & response_out);
  bool get_param_names(const json & request, json & response_out);

  /// Shared dispatch for set_param and delete_param (delete = set to NOT_SET).
  bool send_set_parameters(
    const std::string & node_name, const std::string & param_name, const json & param_value,
    const std::string & rosapi_service, const json & request, json & response_out);

  GenericClient::SharedPtr get_client(const std::string & service, const std::string & type);
  /// get_client + bounded readiness wait; nullptr when the service never became ready.
  GenericClient::SharedPtr get_ready_client(const std::string & service, const std::string & type);
  bool node_exists(const std::string & node_name);

  std::shared_ptr<NodeInterface<>> node_;
  std::function<void(std::string & msg)> send_;
  std::map<std::string, GenericClient::SharedPtr> clients_;
};

}  // namespace rws

#endif  // RWS__ROSAPI_PARAMS_HPP_
