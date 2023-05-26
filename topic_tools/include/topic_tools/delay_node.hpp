// Copyright 2022 AIT Austrian Institute of Technology GmbH
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

#ifndef TOPIC_TOOLS__DELAY_NODE_HPP_
#define TOPIC_TOOLS__DELAY_NODE_HPP_

#include <list>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "topic_tools/tool_base_node_single_sub.hpp"
#include "topic_tools/visibility_control.h"

namespace topic_tools
{
class DelayNode final : public ToolBaseNodeSingleSub
{
public:
  TOPIC_TOOLS_PUBLIC
  explicit DelayNode(const rclcpp::NodeOptions & options);

private:
  void process_message(std::shared_ptr<rclcpp::SerializedMessage> msg) override;

  rclcpp::Duration delay_;
  bool use_wall_clock_;
  std::list<rclcpp::TimerBase::SharedPtr> timers_;
};
}  // namespace topic_tools

#endif  // TOPIC_TOOLS__DELAY_NODE_HPP_
