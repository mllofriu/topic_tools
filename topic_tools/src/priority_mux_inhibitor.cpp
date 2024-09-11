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
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "topic_tools/priority_mux_node_inhibitor.hpp"

/**************************************
 * priority_mux inhibitor entry point *
 * ************************************
 * Parameters:
 *    - time_window (REQUIRED):
 *        Specifies the time (in milliseconds) during which the effect specified in [type] will take effect.
 *    - output_topic (REQUIRED):
 *        Specifies the name of the output topic where the mux republish the messages.
 *          - The inhibitor mux only republish on this topic for the inhibited topic (if the time window is not active).
 *    - input_topics (REQUIRED):
 *        Specifies a vector containing the list of input topics.
 *        The order matters. Ordered from highest to lowest priority.
 * Execution example:
 *    ~ ros2 run topic_tools priority_mux_inhibitor --ros-args -p time_window:=3000 -p output_topic:="/prioritized" -p input_topics:="['/topic_a', '/topic_b']"
 *        Here:
 *          time_window is 3 seconds.
 *          '/topic_a' will have more priority than '/topic_b'.
 *          the node publishing on '/topic_a' inhibits the node publishing on '/topic_b' for 3 seconds
 *************************************************************/

int main(int argc, char * argv[])
{
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  auto options = rclcpp::NodeOptions{};

  if (args.size() >= 3) {
    options.append_parameter_override("time_window", args.at(1));
    options.append_parameter_override("output_topic", args.at(2));
    options.append_parameter_override(
      "input_topics",
      std::vector<std::string>{args.begin() + 3, args.end()});
  }

  auto node = std::make_shared<topic_tools::PriorityMuxNodeInhibitor>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
