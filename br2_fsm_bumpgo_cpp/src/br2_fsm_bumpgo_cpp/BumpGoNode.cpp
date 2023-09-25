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

#include <utility>
#include "br2_fsm_bumpgo_cpp/BumpGoNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace br2_fsm_bumpgo_cpp
{

using namespace std::chrono_literals;
using std::placeholders::_1;  // the placeholder _1 is used in the std::bind function in the scan_sub definition to represent the first argument to the function object that is being created. 

BumpGoNode::BumpGoNode()
: Node("bump_go"),  // what comes after the colon (:) here is the initialization list.  It is used to initialize the member variables of the class before the constructor body executes. This is preferred over initializing the attribute in the constructor body because it can be more efficient, especially for const or reference member variables.
  state_(FORWARD)
{
  // As a general rule, for communications to be compatible, the QoS of the publisher should be reliable, and it is the subscriber who can choose to relax it to be the best effort.
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(),  // (rclcpp::SensorDataQoS() is a packed QoS definition which uses best effort, volatile and appropriate queue size for sensors). 
    std::bind(&BumpGoNode::scan_callback, this, _1));  // scan_callback will be called whenever a new message is received on the topic. The std::bind() function is used to bind the callback function to the BumpGoNode class instance (this) and pass the received message as an argument (_1).

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);  // default QoS is used (which is reliable + volatile)
  timer_ = create_wall_timer(50ms, std::bind(&BumpGoNode::control_cycle, this));

  state_ts_ = now();  // now() gets the current time from rclcpp::Clock
}

void
BumpGoNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);  // the ownership of the memory resource pointed to by the msg unique pointer is transferred to the last_scan_. After the move, the msg pointer no longer owns the memory resource and it's value is set to nullptr.
}

void
BumpGoNode::control_cycle()
{
  // Do nothing until the first sensor read
  if (last_scan_ == nullptr) {return;}

  geometry_msgs::msg::Twist out_vel;

  switch (state_) {
    case FORWARD:
      out_vel.linear.x = SPEED_LINEAR;

      if (check_forward_2_stop()) {
        go_state(STOP);
      }
      if (check_forward_2_back()) {
        go_state(BACK);
      }

      break;
    case BACK:
      out_vel.linear.x = -SPEED_LINEAR;

      if (check_back_2_turn()) {
        go_state(TURN);
      }

      break;
    case TURN:
      out_vel.angular.z = SPEED_ANGULAR;

      if (check_turn_2_forward()) {
        go_state(FORWARD);
      }

      break;
    case STOP:
      if (check_stop_2_forward()) {
        go_state(FORWARD);
      }
      break;
  }

  vel_pub_->publish(out_vel);
}

void
BumpGoNode::go_state(int new_state)
{
  state_ = new_state;
  state_ts_ = now();
}

bool
BumpGoNode::check_forward_2_back()
{
  // going forward when deteting an obstacle
  // at 0.5 meters with the front laser read
  size_t pos = last_scan_->ranges.size() / 2;  // size_t is an unsigned integer type specifically designed to represent sizes and indices of objects in memory. It is a platform-dependent type that is guaranteed to be large enough to represent the size of any object in memory.
  return last_scan_->ranges[pos] < OBSTACLE_DISTANCE;
}

bool
BumpGoNode::check_forward_2_stop()
{
  // Stop if no sensor readings for 1 second
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed > SCAN_TIMEOUT;
}

bool
BumpGoNode::check_stop_2_forward()
{
  // Going forward if sensor readings are available
  // again
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed < SCAN_TIMEOUT;
}

bool
BumpGoNode::check_back_2_turn()
{
  // Going back for 2 seconds
  return (now() - state_ts_) > BACKING_TIME;
}

bool
BumpGoNode::check_turn_2_forward()
{
  // Turning for 2 seconds
  return (now() - state_ts_) > TURNING_TIME;
}

}  // namespace br2_fsm_bumpgo_cpp
