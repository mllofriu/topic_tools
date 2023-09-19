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

#pragma once

#include <memory>
#include <optional>  // NOLINT : https://github.com/ament/ament_lint/pull/324
#include <string>
#include <utility>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "topic_tools/tool_base_node.hpp"
#include "topic_tools/visibility_control.h"

namespace topic_tools
{
struct topic_data
{
  std::optional<std::string> topic_type;
  rclcpp::GenericSubscription::SharedPtr sub;
  std::optional<rclcpp::QoS> qos_profile;
  std::string input_topic;
};

class ToolBaseNodeMultiSub : public ToolBaseNode
{
public:
  TOPIC_TOOLS_PUBLIC
  ToolBaseNodeMultiSub(const std::string & node_name, const rclcpp::NodeOptions & options);

protected:
  virtual void make_subscribe_unsubscribe_decisions();
  int get_topic_index(const std::string& topic_name);
  std::vector<topic_data> topics_data_;

  std::chrono::duration<float> discovery_period_ = std::chrono::milliseconds{100};
  rclcpp::TimerBase::SharedPtr discovery_timer_;
};
}  // namespace topic_tools
