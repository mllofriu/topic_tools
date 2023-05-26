// Copyright 2021 Daisuke Nishimatsu
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

#include <memory>
#include <optional> // NOLINT : https://github.com/ament/ament_lint/pull/324
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "topic_tools/tool_base_node_single_sub.hpp"

namespace topic_tools
{
ToolBaseNodeSingleSub::ToolBaseNodeSingleSub(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: ToolBaseNode(node_name, options)
{
}

void ToolBaseNodeSingleSub::make_subscribe_unsubscribe_decisions()
{
  make_subscribe_unsubscribe_decisions_for_topic(
    input_topic_, sub_, topic_type_, qos_profile_);
}

}  // namespace topic_tools
