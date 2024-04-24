// Copyright 2021 Intelligent Robotics Lab
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

#include "br2_tracking/ObjectDetector.hpp"
#include "br2_tracking/HeadController.hpp"

#include "br2_tracking_msgs/msg/pan_tilt_command.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_detector = std::make_shared<br2_tracking::ObjectDetector>();
  auto node_head_controller = std::make_shared<br2_tracking::HeadController>();
  auto node_tracker = rclcpp::Node::make_shared("tracker");

  auto command_pub = node_tracker->create_publisher<br2_tracking_msgs::msg::PanTiltCommand>(
    "/command", 100);
  auto detection_sub = node_tracker->create_subscription<vision_msgs::msg::Detection2D>(
    "/detection", rclcpp::SensorDataQoS(),
    [command_pub](vision_msgs::msg::Detection2D::SharedPtr msg) {
      br2_tracking_msgs::msg::PanTiltCommand command;
      // By dividing the x-coordinate of the object's center by the width of the image, we are converting the absolute position of the object into a relative position. The result of this operation is a value between 0 and 1, representing the relative horizontal position of the object within the image
      // then, by multiplying by 2, we are scaling the range of the relative position from a value between [0, 1] to a value between [0, 2]
      // Finally, subtracting 1 from the scaled value shifts the range from [0, 2] to [-1, 1]
      command.pan = (msg->bbox.center.x / msg->source_img.width) * 2.0 - 1.0;  
      command.tilt = (msg->bbox.center.y / msg->source_img.height) * 2.0 - 1.0; 
      command_pub->publish(command);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_detector);
  executor.add_node(node_head_controller->get_node_base_interface());
  executor.add_node(node_tracker);

  node_head_controller->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);  // why is this required ? 

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
