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

#include "rws/rosapi_params.hpp"

using rws::json;

TEST(SplitParamName, SplitsAndMakesNodeAbsolute)
{
  std::string node, param;
  ASSERT_TRUE(rws::split_param_name("my_node:my_param", node, param));
  EXPECT_EQ(node, "/my_node");
  EXPECT_EQ(param, "my_param");
}

TEST(SplitParamName, KeepsNamespacedNode)
{
  std::string node, param;
  ASSERT_TRUE(rws::split_param_name("/ns/my_node:tree.leaf", node, param));
  EXPECT_EQ(node, "/ns/my_node");
  EXPECT_EQ(param, "tree.leaf");
}

TEST(SplitParamName, RejectsMalformedNames)
{
  std::string node, param;
  EXPECT_FALSE(rws::split_param_name("no_colon", node, param));
  EXPECT_FALSE(rws::split_param_name(":param_only", node, param));
  EXPECT_FALSE(rws::split_param_name("node_only:", node, param));
  EXPECT_FALSE(rws::split_param_name("", node, param));
}

TEST(ParamValueToJson, ExtractsFieldByType)
{
  json pv = {{"type", 4}, {"string_value", "hello"}, {"integer_value", 0}};
  auto v = rws::param_value_to_json(pv);
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ(*v, json("hello"));

  pv = {{"type", 2}, {"integer_value", 42}};
  v = rws::param_value_to_json(pv);
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ(*v, json(42));

  pv = {{"type", 8}, {"double_array_value", {1.5, 2.5}}};
  v = rws::param_value_to_json(pv);
  ASSERT_TRUE(v.has_value());
  EXPECT_EQ(*v, json({1.5, 2.5}));
}

TEST(ParamValueToJson, NotSetOrUnknownTypeIsEmpty)
{
  EXPECT_FALSE(rws::param_value_to_json(json{{"type", 0}}).has_value());
  EXPECT_FALSE(rws::param_value_to_json(json{{"type", 10}}).has_value());
  EXPECT_FALSE(rws::param_value_to_json(json::object()).has_value());
}

TEST(JsonToParamValue, MapsScalars)
{
  EXPECT_EQ(rws::json_to_param_value(json(true)), (json{{"type", 1}, {"bool_value", true}}));
  EXPECT_EQ(rws::json_to_param_value(json(5)), (json{{"type", 2}, {"integer_value", 5}}));
  EXPECT_EQ(rws::json_to_param_value(json(2.5)), (json{{"type", 3}, {"double_value", 2.5}}));
  EXPECT_EQ(rws::json_to_param_value(json("s")), (json{{"type", 4}, {"string_value", "s"}}));
}

TEST(JsonToParamValue, MapsArrays)
{
  EXPECT_EQ(
    rws::json_to_param_value(json::parse("[true, false]")),
    (json{{"type", 6}, {"bool_array_value", {true, false}}}));
  EXPECT_EQ(
    rws::json_to_param_value(json::parse("[1, 2]")),
    (json{{"type", 7}, {"integer_array_value", {1, 2}}}));
  // A mixed int/float array is promoted to a double array.
  EXPECT_EQ(
    rws::json_to_param_value(json::parse("[1, 2.5]")),
    (json{{"type", 8}, {"double_array_value", {1, 2.5}}}));
  EXPECT_EQ(
    rws::json_to_param_value(json::parse("[\"a\", \"b\"]")),
    (json{{"type", 9}, {"string_array_value", {"a", "b"}}}));
}

TEST(JsonToParamValue, UnmappableValuesBecomeStrings)
{
  EXPECT_EQ(
    rws::json_to_param_value(json::parse("{\"a\": 1}")),
    (json{{"type", 4}, {"string_value", "{\"a\":1}"}}));
  EXPECT_EQ(
    rws::json_to_param_value(json::parse("[\"a\", 1]")),
    (json{{"type", 4}, {"string_value", "[\"a\",1]"}}));
  EXPECT_EQ(
    rws::json_to_param_value(json::parse("null")),
    (json{{"type", 4}, {"string_value", "null"}}));
}
