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
#include "topic_tools/priority_mux_node.hpp"

/**
 * @brief Multiplexer for generic topic types based on priorities.
 * It implements a inhibition mechanism based on the subsumption architecture.
 */

namespace topic_tools
{

class PriorityMuxNodeInhibitor final : public PriorityMuxNode
{
public:
  TOPIC_TOOLS_PUBLIC
  explicit PriorityMuxNodeInhibitor(const rclcpp::NodeOptions & options);

  // Explicitly define a noexcept destructor
  ~PriorityMuxNodeInhibitor() noexcept override = default;

private:
  /**
   * @brief Processes incoming messages and handles them based on their priority.
   * A message will be republished only if it is from the lowest priority topic
   * or there is no active higher priority message within the time window.
   * @param topic_name The name of the topic the message was received from.
   * @param msg The received message.
   */
  void process_message(
    std::string topic_name,
    std::shared_ptr<rclcpp::SerializedMessage> msg) override;
  // The greatest priority value. It corresponds to the lowest priority
  int greatest_priority_value_;
};

}  // namespace topic_tools
