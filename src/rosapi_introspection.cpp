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

#include "rws/rosapi_introspection.hpp"

#include <algorithm>
#include <cstdlib>
#include <set>
#include <utility>

#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rws/typesupport_helpers.hpp"

namespace rws
{

using rosidl_typesupport_introspection_cpp::MessageMember;
using rosidl_typesupport_introspection_cpp::MessageMembers;
using rosidl_typesupport_introspection_cpp::ServiceMembers;

namespace
{

rclcpp::Logger logger() { return rclcpp::get_logger("rws_rosapi_introspection"); }

// Same primitive naming as generate_message_meta (byte/char folded into uint8).
const char * primitive_type_name(uint8_t type_id)
{
  switch (type_id) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
      return "bool";
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
      return "uint8";
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      return "int8";
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32:
      return "float32";
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64:
      return "float64";
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      return "int16";
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      return "uint16";
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      return "int32";
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      return "uint32";
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      return "int64";
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
      return "uint64";
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
      return "string";
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
      return "wstring";
    default:
      return "unknown";
  }
}

// Default-value string shown in TypeDef examples, like the python rosapi's
// str() of a default-constructed field.
std::string example_for_member(const MessageMember & member)
{
  if (member.is_array_) {
    return "[]";
  }
  switch (member.type_id_) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
      return "{}";
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
      return "False";
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
      return "";
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64:
      return "0.0";
    default:
      return "0";
  }
}

// rosapi fieldarraylen: -1 scalar, 0 variable-length array, N fixed array.
int32_t array_len_for_member(const MessageMember & member)
{
  if (!member.is_array_) {
    return -1;
  }
  if (member.array_size_ && !member.is_upper_bound_) {
    return static_cast<int32_t>(member.array_size_);
  }
  return 0;
}

void members_to_typedefs(
  const MessageMembers * members, json & defs, std::set<std::string> & seen,
  bool rosbridge_compatible)
{
  std::string type = get_type_from_message_members(members);
  if (seen.count(type)) {
    return;
  }
  seen.insert(type);

  json td = {
    {"type", type},
    {"fieldnames", json::array()},
    {"fieldtypes", json::array()},
    {"fieldarraylen", json::array()},
    {"examples", json::array()},
    // Introspection typesupport doesn't expose message constants.
    {"constnames", json::array()},
    {"constvalues", json::array()},
  };

  std::vector<const MessageMembers *> nested;
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const auto & member = members->members_[i];
    std::string name = member.name_;
    if (name == "structure_needs_at_least_one_member") {
      continue;
    }

    // Match the secs/nsecs mapping the fork applies to actual messages.
    if (rosbridge_compatible && type == "builtin_interfaces/msg/Time") {
      if (name == "sec") {
        name = "secs";
      } else if (name == "nanosec") {
        name = "nsecs";
      }
    }

    std::string field_type;
    if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
      auto sub_members = static_cast<const MessageMembers *>(member.members_->data);
      field_type = get_type_from_message_members(sub_members);
      nested.push_back(sub_members);
    } else {
      field_type = primitive_type_name(member.type_id_);
    }

    td["fieldnames"].push_back(name);
    td["fieldtypes"].push_back(field_type);
    td["fieldarraylen"].push_back(array_len_for_member(member));
    td["examples"].push_back(example_for_member(member));
  }

  defs.push_back(td);
  for (const auto * sub : nested) {
    members_to_typedefs(sub, defs, seen, rosbridge_compatible);
  }
}

// Accept both "pkg/Type" and "pkg/msg/Type" spellings from clients.
std::string normalize_type(const std::string & type, const std::string & middle)
{
  auto first = type.find('/');
  if (first != std::string::npos && type.find('/', first + 1) == std::string::npos) {
    return type.substr(0, first) + "/" + middle + "/" + type.substr(first + 1);
  }
  return type;
}

// "/ns/node" -> ("/ns", "node"); "/node" -> ("/", "node")
std::pair<std::string, std::string> split_node_fqn(const std::string & fqn)
{
  auto pos = fqn.rfind('/');
  if (pos == std::string::npos) {
    return {"/", fqn};
  }
  std::string ns = pos == 0 ? "/" : fqn.substr(0, pos);
  return {ns, fqn.substr(pos + 1)};
}

bool ends_with(const std::string & s, const std::string & suffix)
{
  return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

}  // namespace

json typedefs_for_message(const std::string & msg_type, bool rosbridge_compatible)
{
  std::string type = normalize_type(msg_type, "msg");
  auto library = rws::get_typesupport_library(type, rws::ts_identifier);
  auto ts = rclcpp::get_message_typesupport_handle(type, rws::ts_identifier, *library);
  auto members = static_cast<const MessageMembers *>(ts->data);

  json defs = json::array();
  std::set<std::string> seen;
  members_to_typedefs(members, defs, seen, rosbridge_compatible);
  return defs;
}

json typedefs_for_service(const std::string & srv_type, bool request, bool rosbridge_compatible)
{
  std::string type = normalize_type(srv_type, "srv");
  auto library = rws::get_typesupport_library(type, rws::ts_identifier);
  auto ts = rws::get_service_typesupport_handle(type, rws::ts_identifier, *library);
  auto srv_members = static_cast<const ServiceMembers *>(ts->data);
  auto members = request ? srv_members->request_members_ : srv_members->response_members_;

  json defs = json::array();
  std::set<std::string> seen;
  members_to_typedefs(members, defs, seen, rosbridge_compatible);
  return defs;
}

std::vector<std::string> filter_action_servers(const std::vector<std::string> & topics)
{
  static const std::string kFeedback = "/_action/feedback";
  static const std::string kStatus = "/_action/status";

  std::set<std::string> with_feedback, with_status;
  for (const auto & topic : topics) {
    if (ends_with(topic, kFeedback)) {
      with_feedback.insert(topic.substr(0, topic.size() - kFeedback.size()));
    } else if (ends_with(topic, kStatus)) {
      with_status.insert(topic.substr(0, topic.size() - kStatus.size()));
    }
  }

  std::vector<std::string> servers;
  for (const auto & ns : with_feedback) {
    if (!ns.empty() && with_status.count(ns)) {
      servers.push_back(ns);
    }
  }
  return servers;
}

RosapiIntrospection::RosapiIntrospection(
  std::shared_ptr<NodeInterface<>> node, bool rosbridge_compatible)
: node_(node), rosbridge_compatible_(rosbridge_compatible)
{
}

bool RosapiIntrospection::handle_service_call(const json & request, json & response_out)
{
  std::string service = request["service"];
  const json args =
    request.contains("args") && request["args"].is_object() ? request["args"] : json::object();

  try {
    if (service == "/rosapi/services") {
      response_out["values"]["services"] = json::array();
      for (const auto & entry : node_->get_service_names_and_types()) {
        response_out["values"]["services"].push_back(entry.first);
      }
      response_out["result"] = true;
      return true;
    }

    if (service == "/rosapi/topics_for_type") {
      std::string queried = normalize_type(args.value("type", ""), "msg");
      response_out["values"]["topics"] = json::array();
      for (const auto & entry : node_->get_topic_names_and_types()) {
        if (std::find(entry.second.begin(), entry.second.end(), queried) != entry.second.end()) {
          response_out["values"]["topics"].push_back(entry.first);
        }
      }
      response_out["result"] = true;
      return true;
    }

    if (service == "/rosapi/action_servers") {
      std::vector<std::string> topics;
      for (const auto & entry : node_->get_topic_names_and_types()) {
        topics.push_back(entry.first);
      }
      response_out["values"]["action_servers"] = filter_action_servers(topics);
      response_out["result"] = true;
      return true;
    }

    if (service == "/rosapi/service_providers" || service == "/rosapi/service_node") {
      // Matched by service *name* (the python rosapi accidentally matched
      // service_providers by type; the .srv field is a name, so name it is).
      std::string queried = args.value("service", "");
      std::vector<std::string> providers;
      for (const auto & fqn : node_->get_node_names()) {
        auto [ns, name] = split_node_fqn(fqn);
        try {
          auto services = node_->get_service_names_and_types_by_node(name, ns);
          if (services.count(queried)) {
            providers.push_back(fqn);
          }
        } catch (const std::exception & e) {
          RCLCPP_WARN(logger(), "%s: skipping %s: %s", service.c_str(), fqn.c_str(), e.what());
        }
      }
      if (service == "/rosapi/service_providers") {
        response_out["values"]["providers"] = providers;
      } else {
        response_out["values"]["node"] = providers.empty() ? "" : providers[0];
      }
      response_out["result"] = true;
      return true;
    }

    if (service == "/rosapi/get_time") {
      int64_t ns = node_->now().nanoseconds();
      json time = {
        {rosbridge_compatible_ ? "secs" : "sec", ns / 1000000000},
        {rosbridge_compatible_ ? "nsecs" : "nanosec", ns % 1000000000},
      };
      response_out["values"]["time"] = time;
      response_out["result"] = true;
      return true;
    }

    if (service == "/rosapi/get_ros_version") {
      const char * distro = std::getenv("ROS_DISTRO");
      response_out["values"]["version"] = 2;
      response_out["values"]["distro"] = distro ? distro : "";
      response_out["result"] = true;
      return true;
    }

    if (service == "/rosapi/message_details") {
      response_out["values"]["typedefs"] =
        typedefs_for_message(args.value("type", ""), rosbridge_compatible_);
      response_out["result"] = true;
      return true;
    }

    if (
      service == "/rosapi/service_request_details" ||
      service == "/rosapi/service_response_details") {
      bool is_request = service == "/rosapi/service_request_details";
      response_out["values"]["typedefs"] =
        typedefs_for_service(args.value("type", ""), is_request, rosbridge_compatible_);
      response_out["result"] = true;
      return true;
    }
  } catch (const std::exception & e) {
    response_out["error"] = "Failed to handle " + service + ": " + e.what();
    RCLCPP_ERROR(logger(), "%s", response_out["error"].get<std::string>().c_str());
    return true;
  }

  return false;
}

}  // namespace rws
