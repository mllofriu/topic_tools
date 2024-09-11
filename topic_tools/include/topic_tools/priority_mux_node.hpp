// Copyright 2023 Martin Llofrui
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

#pragma once

#include <memory>
#include <optional>  // NOLINT : https://github.com/ament/ament_lint/pull/324
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "topic_tools/tool_base_node_multi_sub.hpp"

/**
 * @brief Multiplexer for generic topic types based on priorities.
 */

namespace topic_tools
{

class PriorityMuxNode : public ToolBaseNodeMultiSub
{
public:
  TOPIC_TOOLS_PUBLIC
  explicit PriorityMuxNode(const rclcpp::NodeOptions & options);

  // Explicitly define a noexcept destructor
  ~PriorityMuxNode() noexcept override = default;

private:
  /**
   * @brief Processes incoming messages and handles them based on their priority.
   * @param topic_name The name of the topic the message was received from.
   * @param msg The received message.
   */
  void process_message(
    std::string topic_name,
    std::shared_ptr<rclcpp::SerializedMessage> msg) override;

protected:
  bool skip_message(const int topic_prio);
  // The index (priority) of the previous topic
  std::optional<int> prev_topic_index_;
  // The timestamp of the last received topic
  rclcpp::Time last_received_;
  // The time during the mechanism will take effect
  rclcpp::Duration time_window_duration_;
  std::mutex mutex_;
};

}  // namespace topic_tools
