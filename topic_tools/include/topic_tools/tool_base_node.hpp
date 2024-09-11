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

#ifndef TOPIC_TOOLS__TOOL_BASE_NODE_HPP_
#define TOPIC_TOOLS__TOOL_BASE_NODE_HPP_

#include <memory>
#include <optional>  // NOLINT : https://github.com/ament/ament_lint/pull/324
#include <string>
#include <utility>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "topic_tools/visibility_control.h"

namespace topic_tools
{
class ToolBaseNode : public rclcpp::Node
{
public:
  TOPIC_TOOLS_PUBLIC
  ToolBaseNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options);

protected:
  virtual void process_message(
    std::string topic_name,
    std::shared_ptr<rclcpp::SerializedMessage> msg
  ) = 0;

  virtual void make_subscribe_unsubscribe_decisions_for_topic(
    const std::string & topic_name,
    rclcpp::GenericSubscription::SharedPtr & sub,
    std::optional<std::string> & topic_type,
    std::optional<rclcpp::QoS> & qos_profile
  );
  virtual void make_subscribe_unsubscribe_decisions() = 0;

  std::string output_topic_;
  bool lazy_;
  rclcpp::GenericPublisher::SharedPtr pub_;
  std::mutex pub_mutex_;

private:
  /// Returns an optional pair <topic type, QoS profile> of the first found source publishing
  /// on `topic_name` if at least one source is found
  std::optional<std::pair<std::string, rclcpp::QoS>> try_discover_source(
    const std::string & topic_name
  ) const;

  std::chrono::duration<float> discovery_period_ = std::chrono::milliseconds{100};
  rclcpp::TimerBase::SharedPtr discovery_timer_;
};
}  // namespace topic_tools

#endif  // TOPIC_TOOLS__TOOL_BASE_NODE_HPP_
