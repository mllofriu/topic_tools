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
#include "topic_tools/priority_mux_node_inhibitor.hpp"

namespace topic_tools
{
PriorityMuxNodeInhibitor::PriorityMuxNodeInhibitor(const rclcpp::NodeOptions & options)
: PriorityMuxNode(options)
{
  greatest_priority_value_ = static_cast<int>(topics_data_.size()) - 1;
}

void PriorityMuxNodeInhibitor::process_message(
  std::string topic_name,
  std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  // Get the topic priority
  int prio = get_topic_index(topic_name);
  assert(prio != -1 && "The topic was not found in the expected list of topics");

  if (!skip_message(prio)) {
    prev_topic_index_ = prio;
    last_received_ = rclcpp::Clock{}.now();
    // Let the message pass only if the topic is the inhibited
    if (prio == greatest_priority_value_) {
      // At this point, inhibition has ended or it hasn't started yet
      pub_->publish(*msg);
    }
  }
}

}  // namespace topic_tools

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(topic_tools::PriorityMuxNodeInhibitor)
