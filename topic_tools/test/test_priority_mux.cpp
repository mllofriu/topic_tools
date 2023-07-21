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
#include <string>
#include <chrono>
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "topic_tools/priority_mux_node.hpp"
#include "test_topic_tool.hpp"

class PriorityMuxTest : public TestTopicToolMultiSub
{
public:
  void SetUp()
  {
    TestTopicToolMultiSub::SetUp();

    auto options = rclcpp::NodeOptions{};
    options.append_parameter_override("output_topic", get_target_output_topic());
    options.append_parameter_override("input_topics", get_target_input_topics());
    target_node_ = std::make_shared<topic_tools::PriorityMuxNode>(options);

    std::function<void(const std_msgs::msg::String::SharedPtr)> validator =
      [](const std_msgs::msg::String::SharedPtr msg) {
        ASSERT_EQ(msg->data, "not dropped");
      };
    set_msg_validator(validator);
  }

  void publish_and_check(
    std::string msg_content,
    int publisher_index)
  {
    TestTopicToolMultiSub::publish_and_check(msg_content, publisher_index, target_node_);
  }

private:
  std::shared_ptr<rclcpp::Node> target_node_;
};

TEST_F(PriorityMuxTest, MessagesToHighestPriorityTopicArrives) {
  publish_and_check("not dropped", 0);

  ASSERT_EQ(get_received_msgs(), 1);
}

TEST_F(PriorityMuxTest, MessagesToNonPriorityTopicArriveIfNoOtherTopicsAreThere) {
  publish_and_check("not dropped", 1);

  ASSERT_EQ(get_received_msgs(), 1);
}

TEST_F(PriorityMuxTest, MessagesToNonPriorityTopicDontArriveIfHigherPrioTopicsArePublishedBefore) {
  publish_and_check("not dropped", 0);
  publish_and_check("dropped", 1);

  ASSERT_EQ(get_received_msgs(), 1);
}
