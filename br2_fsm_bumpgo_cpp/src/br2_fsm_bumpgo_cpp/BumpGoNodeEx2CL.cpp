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
#include "br2_fsm_bumpgo_cpp/BumpGoNodeEx2CL.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace br2_fsm_bumpgo_cpp
{

using namespace std::chrono_literals;
using std::placeholders::_1;  // the placeholder _1 is used in the std::bind function in the scan_sub definition to represent the first argument to the function object that is being created. 

BumpGoNodeEx2CL::BumpGoNodeEx2CL()
: Node("bump_go_ex2_ol"),  // what comes after the colon (:) here is the initialization list.  It is used to initialize the member variables of the class before the constructor body executes. This is preferred over initializing the attribute in the constructor body because it can be more efficient, especially for const or reference member variables.
  state_(FORWARD), rotation_duration(0)
{
  // As a general rule, for communications to be compatible, the QoS of the publisher should be reliable, and it is the subscriber who can choose to relax it to be the best effort.
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(),  // (rclcpp::SensorDataQoS() is a packed QoS definition which uses best effort, volatile and appropriate queue size for sensors). 
    std::bind(&BumpGoNodeEx2CL::scan_callback, this, _1));  // scan_callback will be called whenever a new message is received on the topic. The std::bind() function is used to bind the callback function to the BumpGoNode class instance (this) and pass the received message as an argument (_1).

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);  // default QoS is used (which is reliable + volatile)
  timer_ = create_wall_timer(50ms, std::bind(&BumpGoNodeEx2CL::control_cycle, this));

  state_ts_ = now();  // now() gets the current time from rclcpp::Clock
}

void
BumpGoNodeEx2CL::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);  // the ownership of the memory resource pointed to by the msg unique pointer is transferred to the last_scan_. After the move, the msg pointer no longer owns the memory resource and it's value is set to nullptr.
}

void
BumpGoNodeEx2CL::control_cycle()
{
  RCLCPP_INFO(this->get_logger(), "Current state: %d", state_);
  // Do nothing until the first sensor read
  if (last_scan_ == nullptr) {return;}

  geometry_msgs::msg::Twist out_vel;

  switch (state_) {
    case FORWARD:
      out_vel.linear.x = SPEED_LINEAR;

      if (check_forward_2_stop()) {
        go_state(STOP);
      }
      if (check_forward_2_back()) {  // in case of front obstacle
        obstacle_direction = "front";
        go_state(BACK);
      }

      // Added code for exercise 1
      if (check_forward_2_turn_right()){  // if obstacle is detected on the left
        obstacle_direction = "left";
        go_state(BACK);
      }
      if (check_forward_2_turn_left()){  // if obstacle is detected on the right
        obstacle_direction = "right";
        go_state(BACK);
      }

      break;
    case BACK:
      out_vel.linear.x = -SPEED_LINEAR;

      if (check_back_2_turn()) {
        if (obstacle_direction == "front"  || obstacle_direction == "left"){
          go_state(TURN_RIGHT);
        }
        else{
        go_state(TURN);
        }
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

    // added code
    case TURN_RIGHT:
      out_vel.angular.z = - SPEED_ANGULAR;

      if (check_turn_2_forward()) {  
        go_state(FORWARD);
      }
      break;
  }

  vel_pub_->publish(out_vel);
}

void
BumpGoNodeEx2CL::go_state(int new_state)
{
  state_ = new_state;
  state_ts_ = now();
}

bool
BumpGoNodeEx2CL::check_forward_2_back()
{
  // going forward when deteting an obstacle
  // at 0.5 meters with the front laser read
  size_t pos = last_scan_->ranges.size() / 2;  // size_t is an unsigned integer type specifically designed to represent sizes and indices of objects in memory. It is a platform-dependent type that is guaranteed to be large enough to represent the size of any object in memory.
  RCLCPP_INFO(this->get_logger(), "Front laser measurement: %f", last_scan_->ranges[pos]);
  return last_scan_->ranges[pos] < OBSTACLE_DISTANCE;
}

bool
BumpGoNodeEx2CL::check_forward_2_stop()
{
  // Stop if no sensor readings for 1 second
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed > SCAN_TIMEOUT;
}

bool
BumpGoNodeEx2CL::check_stop_2_forward()
{
  // Going forward if sensor readings are available
  // againfind_farthest_obstact_angle_and_rotate
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed < SCAN_TIMEOUT;
}

bool
BumpGoNodeEx2CL::check_back_2_turn()
{
  // Going back for 2 seconds
  return (now() - state_ts_) > BACKING_TIME;
}

bool
BumpGoNodeEx2CL::check_turn_2_forward()
{
  // added code
  // auto time_since_state = now() - state_ts_;
  // bool done_turning = time_since_state > rotation_duration;
  // return done_turning;
  size_t front_index = last_scan_->ranges.size() / 2;
  RCLCPP_INFO(this->get_logger(), "Front laser measurement during rotation: %f", last_scan_->ranges[front_index]);
  return last_scan_->ranges[front_index] > OBSTACLE_FREE_DISTANCE;
}

// added code
bool
BumpGoNodeEx2CL::check_forward_2_turn_left()
{
  // checking if there is an obstacle on the right, and turning left if this is the case
  size_t right_diag = last_scan_->ranges.size() * (1.0 / 3.0);
  RCLCPP_INFO(this->get_logger(), "Right diagonal measurement: %f", last_scan_->ranges[right_diag]);
  return last_scan_->ranges[right_diag] < OBSTACLE_DISTANCE;
}

bool
BumpGoNodeEx2CL::check_forward_2_turn_right()
{
  // checking if there is an obstacle on the left, and turning right if this is the case
  size_t left_diag = last_scan_->ranges.size() * (2.0 / 3.0);
  RCLCPP_INFO(this->get_logger(), "Left diagonal measurement: %f", last_scan_->ranges[left_diag]);
  return last_scan_->ranges[left_diag] < OBSTACLE_DISTANCE;
}

size_t 
BumpGoNodeEx2CL::find_farthest_obstact_angle_index()
{
  // Finding the rotation angle for the farthest obstacle
  // Finding the total number of measurements
  size_t n = last_scan_->ranges.size();

  // Finding the largest measurement
  auto max_element = std::max_element(last_scan_->ranges.begin(), last_scan_->ranges.end());

  // Finding the index of the largest measurement
  auto max_index = std::distance(last_scan_->ranges.begin(), max_element);
  RCLCPP_INFO(this->get_logger(), "The farthest obstacle is at index: %d", max_index);

  return max_index;

}

rclcpp::Duration
BumpGoNodeEx2CL::calculate_rotation_duration()
{
  if (best_angle_index <= last_scan_->ranges.size() / 2){
    auto rot_time = rclcpp::Duration::from_seconds((((last_scan_->ranges.size()/2) - best_angle_index) * last_scan_->angle_increment) / SPEED_ANGULAR);
    RCLCPP_INFO(this->get_logger(), "Calculated Rotation Time: %f", rot_time);
    return rot_time;
  }
  else {
    auto rot_time = rclcpp::Duration::from_seconds(((best_angle_index - (last_scan_->ranges.size()/2)) * last_scan_->angle_increment) / SPEED_ANGULAR);
    RCLCPP_INFO(this->get_logger(), "Calculated Rotation Time: %f", rot_time);
    return rot_time;
  }
}

} 
