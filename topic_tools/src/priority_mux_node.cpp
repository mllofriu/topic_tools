// Copyright 2023 Martin Llofriu
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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "topic_tools/priority_mux_node.hpp"

namespace topic_tools
{
PriorityMuxNode::PriorityMuxNode(const rclcpp::NodeOptions & options)
: ToolBaseNodeMultiSub("priority_mux", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Override output topic's name with this node's preference
  output_topic_ = declare_parameter("output_topic", "prioritized");
  std::vector<std::string> input_topics =
    declare_parameter<std::vector<std::string>>("input_topics");
  for (const std::string & input_topic : input_topics) {
    topic_data td;
    td.input_topic = input_topic;
    topics_data_.push_back(td);
  }

  make_subscribe_unsubscribe_decisions();
}

void PriorityMuxNode::process_message(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  pub_->publish(*msg);
}

}  // namespace topic_tools

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(topic_tools::PriorityMuxNode)
