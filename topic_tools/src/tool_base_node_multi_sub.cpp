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
#include "topic_tools/tool_base_node_multi_sub.hpp"

namespace topic_tools
{
ToolBaseNodeMultiSub::ToolBaseNodeMultiSub(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: ToolBaseNode(node_name, options)
{
}

void ToolBaseNodeMultiSub::make_subscribe_unsubscribe_decisions()
{
  for (topic_data & topic_data : topics_data_) {
    make_subscribe_unsubscribe_decisions_for_topic(
      topic_data.input_topic,
      topic_data.sub,
      topic_data.topic_type,
      topic_data.qos_profile
    );
  }
}

int ToolBaseNodeMultiSub::get_topic_index(const std::string & topic_name)
{
  int index = -1;
  for (size_t i = 0; i < topics_data_.size(); i++) {
    const topic_data & td = topics_data_[i];
    if (td.input_topic == topic_name) {
      index = i;
    }
  }
  return index;
}

}  // namespace topic_tools
