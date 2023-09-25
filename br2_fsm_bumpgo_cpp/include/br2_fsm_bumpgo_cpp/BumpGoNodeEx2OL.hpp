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

#ifndef BR2_FSM_BUMPGO_CPP__BUMPGONODE_HPP_  // directive checks if the given token (BR2_FSM_BUMPGO_CPP__BUMPGONODE_HPP_) has not been defined earlier in the file or in an included file. If the token is not defined, the code between the #ifndef and the #endif directives will be included in the compilation process.
#define BR2_FSM_BUMPGO_CPP__BUMPGONODE_HPP_  // The #define directive defines the token, so the next time the header file is included, the #ifndef condition will be false, and the code between the #ifndef and the #endif directives will not be included again. This prevents multiple inclusions of the same header file, which can cause compilation errors due to redefinition of classes, functions, or variables.

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "algorithm"

#include "rclcpp/rclcpp.hpp"

namespace br2_fsm_bumpgo_cpp_ex2_ol  // This helps to keep the code organized and reduces the likelihood of naming conflicts with other parts of the program or external libraries
{

using namespace std::chrono_literals;  // NOLINT

class BumpGoNodeEx2OL : public rclcpp::Node
{
public:
  BumpGoNodeEx2OL();

private:
  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
  void control_cycle();

  static const int FORWARD = 0;  // the static keyword means that the attribute is shared by all instances of the class, rather than being specific to each instance. This means that there is only one copy of the attribute, regardless of how many objects of the class are created.
  static const int BACK = 1;
  static const int TURN = 2;  // Turning left
  static const int STOP = 3;
  static const int TURN_RIGHT = 4;  // added code

  int state_;
  rclcpp::Time state_ts_;
  size_t best_angle_index;  // added code
  rclcpp::Duration rotation_duration; // added code

  void go_state(int new_state);
  bool check_forward_2_back();
  bool check_forward_2_stop();
  bool check_back_2_turn();
  bool check_turn_2_forward();
  bool check_stop_2_forward();
  bool check_forward_2_turn_right(); // added code
  bool check_forward_2_turn_left(); // added code
  size_t find_farthest_obstact_angle_index(); // added code
  rclcpp::Duration calculate_rotation_duration(); // added code

  const rclcpp::Duration TURNING_TIME {2s};  // {} is called Uniform Initialization Syntax. It makes the code easier to read, prevents narrow conversions and it can be used to initialize aggregates (arrays, structs) 
  const rclcpp::Duration BACKING_TIME {2s};
  const rclcpp::Duration SCAN_TIMEOUT {1s};

  static constexpr float SPEED_LINEAR = 0.3f;  // the constexpr keyword means that the attribute is a compile-time constant that can be used in constant expression. This means that the value of the attribute is known at compile time and can be used in contexts where a constant expression is required, such as array sizes and template arguments.
  static constexpr float SPEED_ANGULAR = 0.3f;  // the suffix `f` means that we are explicitly stating that 0.3 is of type float (without the f suffix, 0.3 would be interpreted as a type double and will implicitly be converted to float). So, using the `f` suffix can help avoid potential issues related to implicit conversions between float and double types.
  static constexpr float OBSTACLE_DISTANCE = 1.0f;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;  // declares a shared pointer to an instance of the rclcpp::Subscription class, which is a template class specialized for the sensor_msgs::msg::LaserScan message type.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;  // declares a shared pointer to an instance of the rclcpp::TimerBase class.

  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
};

}  // namespace br2_fsm_bumpgo_cpp

#endif  // BR2_FSM_BUMPGO_CPP__BUMPGONODE_HPP_
