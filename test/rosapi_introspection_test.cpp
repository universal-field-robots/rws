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

#include <gtest/gtest.h>

#include "rws/rosapi_introspection.hpp"

using rws::json;

TEST(FilterActionServers, FindsServersWithFeedbackAndStatus)
{
  std::vector<std::string> topics = {
    "/fibonacci/_action/feedback",
    "/fibonacci/_action/status",
    "/fibonacci/_action/goal",
    "/only_feedback/_action/feedback",
    "/only_status/_action/status",
    "/chatter",
  };
  EXPECT_EQ(rws::filter_action_servers(topics), std::vector<std::string>({"/fibonacci"}));
}

TEST(FilterActionServers, EmptyWhenNoActionTopics)
{
  EXPECT_TRUE(rws::filter_action_servers({"/chatter", "/rosout"}).empty());
}

TEST(TypedefsForMessage, HeaderWithNestedTime)
{
  json defs = rws::typedefs_for_message("std_msgs/msg/Header");
  ASSERT_EQ(defs.size(), 2u);

  const json & header = defs[0];
  EXPECT_EQ(header["type"], "std_msgs/msg/Header");
  EXPECT_EQ(header["fieldnames"], json({"stamp", "frame_id"}));
  EXPECT_EQ(header["fieldtypes"], json({"builtin_interfaces/msg/Time", "string"}));
  EXPECT_EQ(header["fieldarraylen"], json({-1, -1}));
  EXPECT_EQ(header["constnames"], json::array());

  const json & time = defs[1];
  EXPECT_EQ(time["type"], "builtin_interfaces/msg/Time");
  EXPECT_EQ(time["fieldnames"], json({"sec", "nanosec"}));
  EXPECT_EQ(time["fieldtypes"], json({"int32", "uint32"}));
}

TEST(TypedefsForMessage, RosbridgeCompatibleRenamesTimeFields)
{
  json defs = rws::typedefs_for_message("std_msgs/msg/Header", true);
  ASSERT_EQ(defs.size(), 2u);
  EXPECT_EQ(defs[1]["fieldnames"], json({"secs", "nsecs"}));
}

TEST(TypedefsForMessage, AcceptsRos1StyleTypeName)
{
  json defs = rws::typedefs_for_message("std_msgs/Header");
  ASSERT_FALSE(defs.empty());
  EXPECT_EQ(defs[0]["type"], "std_msgs/msg/Header");
}

TEST(TypedefsForMessage, ArrayLengths)
{
  // covariance is float64[36] (fixed), so fieldarraylen must carry 36.
  json defs = rws::typedefs_for_message("geometry_msgs/msg/PoseWithCovariance");
  ASSERT_FALSE(defs.empty());
  EXPECT_EQ(defs[0]["fieldnames"], json({"pose", "covariance"}));
  EXPECT_EQ(defs[0]["fieldarraylen"], json({-1, 36}));
}

TEST(TypedefsForService, RequestAndResponse)
{
  json req = rws::typedefs_for_service("std_srvs/srv/SetBool", true);
  ASSERT_EQ(req.size(), 1u);
  EXPECT_EQ(req[0]["fieldnames"], json({"data"}));
  EXPECT_EQ(req[0]["fieldtypes"], json({"bool"}));

  json res = rws::typedefs_for_service("std_srvs/srv/SetBool", false);
  ASSERT_EQ(res.size(), 1u);
  EXPECT_EQ(res[0]["fieldnames"], json({"success", "message"}));
  EXPECT_EQ(res[0]["fieldtypes"], json({"bool", "string"}));
}

TEST(TypedefsForService, EmptyRequestHasNoFields)
{
  json req = rws::typedefs_for_service("std_srvs/srv/Trigger", true);
  ASSERT_EQ(req.size(), 1u);
  EXPECT_EQ(req[0]["fieldnames"], json::array());
}
