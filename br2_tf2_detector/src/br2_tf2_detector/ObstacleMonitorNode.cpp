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

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <memory>
#include <cmath>

#include "br2_tf2_detector/ObstacleMonitorNode.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

namespace br2_tf2_detector
{

using namespace std::chrono_literals;

ObstacleMonitorNode::ObstacleMonitorNode()
: Node("obstacle_monitor"),
  tf_buffer_(),
  tf_listener_(tf_buffer_), 
  THRESHOLD_DISTANCE(2.0)
{
  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("obstacle_marker", 1);  // original topic
  odom_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("odom_obstacle_marker", 1);
  head_2_link_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("head_2_link_obstacle_marker", 1);

  timer_ = create_wall_timer(
    500ms, std::bind(&ObstacleMonitorNode::control_cycle, this));
}

void
ObstacleMonitorNode::control_cycle()
{
  geometry_msgs::msg::TransformStamped robot2obstacle;
  geometry_msgs::msg::TransformStamped odom2obstacle;
  geometry_msgs::msg::TransformStamped head_2_link2obstacle;

  try {
    robot2obstacle = tf_buffer_.lookupTransform("base_footprint", "detected_obstacle", tf2::TimePointZero);  // tf2::TimePointZero represents the latest common time between the two frames
    odom2obstacle = tf_buffer_.lookupTransform("odom", "detected_obstacle", tf2::TimePointZero);
    head_2_link2obstacle = tf_buffer_.lookupTransform("head_2_link", "detected_obstacle", tf2::TimePointZero);

  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "Obstacle transform not found: %s", ex.what());
    return;
  }

  // Obstacle coordinates in base_footprint frame
  double x = robot2obstacle.transform.translation.x;
  double y = robot2obstacle.transform.translation.y;
  double z = robot2obstacle.transform.translation.z;
  double theta = atan2(y, x);

  // Obstacle coordinates in odom frame
  double x_in_odom = odom2obstacle.transform.translation.x;
  double y_in_odom = odom2obstacle.transform.translation.y;
  double z_in_odom = odom2obstacle.transform.translation.z;
  double theta_in_odom = atan2(y_in_odom, x_in_odom);

  // Obstacle coordinates in head_2_link frame
  double x_in_head_2_link = head_2_link2obstacle.transform.translation.x;
  double y_in_head_2_link = head_2_link2obstacle.transform.translation.y;
  double z_in_head_2_link = head_2_link2obstacle.transform.translation.z;
  double theta_in_head_2_link = atan2(y_in_head_2_link, x_in_head_2_link);

  RCLCPP_INFO(get_logger(), "Obstacle detected in odom frame at           (%lf m, %lf m, , %lf m) = %lf rads", x_in_odom, y_in_odom, z_in_odom, theta_in_odom);  // `lf` stands for long float, which represents a double-floating point number
  RCLCPP_INFO(get_logger(), "Obstacle detected in base_footprint frame at (%lf m, %lf m, , %lf m) = %lf rads", x, y, z, theta); 
  RCLCPP_INFO(get_logger(), "Obstacle detected in head_2_link frame at    (%lf m, %lf m, , %lf m) = %lf rads", x_in_head_2_link, y_in_head_2_link, z_in_head_2_link, theta_in_head_2_link); 


  visualization_msgs::msg::Marker obstacle_arrow;
  obstacle_arrow.header.frame_id = "base_footprint";
  obstacle_arrow.header.stamp = now();
  obstacle_arrow.type = visualization_msgs::msg::Marker::ARROW;
  obstacle_arrow.action = visualization_msgs::msg::Marker::ADD;  // Adding the arrow marker to the visualization
  obstacle_arrow.lifetime = rclcpp::Duration(1s);

  geometry_msgs::msg::Point start;
  start.x = 0.0;
  start.y = 0.0;
  start.z = 0.0;
  geometry_msgs::msg::Point end;
  end.x = x;
  end.y = y;
  end.z = z;
  obstacle_arrow.points = {start, end};  // The list initialization syntax {} can be used to initialize a vector (obstacle_arrow.points) with a list of elements.

  // Checking distance between base_footprint and obstacle
  double dist = sqrt( pow(x, 2) + pow(y, 2) );

  // Changing arrow color based on distance between base_footprint and obstacle
  if (dist > THRESHOLD_DISTANCE){
    obstacle_arrow.color.r = 0.0;
    obstacle_arrow.color.g = 1.0;
    obstacle_arrow.color.b = 0.0;
  }else{
    obstacle_arrow.color.r = 1.0;
    obstacle_arrow.color.g = 0.0;
    obstacle_arrow.color.b = 0.0;
  } 

  
  obstacle_arrow.color.a = 1.0;  // Setting the alpha=1.0 since it is 0.0 by default. This will allow us to see the arrow in RViz2.

  obstacle_arrow.scale.x = 0.02;
  obstacle_arrow.scale.y = 0.1;
  obstacle_arrow.scale.z = 0.1;

  // Creating markers for obstacles detected in odom frame
  visualization_msgs::msg::Marker odom_arrow;
  odom_arrow.header.frame_id = "odom";
  odom_arrow.header.stamp = now();
  odom_arrow.type = visualization_msgs::msg::Marker::ARROW;
  odom_arrow.action = visualization_msgs::msg::Marker::ADD;  // Adding the arrow marker to the visualization
  odom_arrow.lifetime = rclcpp::Duration(1s);
  geometry_msgs::msg::Point start_odom;
  start_odom.x = 0.0;
  start_odom.y = 0.0;
  start_odom.z = 0.0;
  geometry_msgs::msg::Point end_odom;
  end_odom.x = x_in_odom;
  end_odom.y = y_in_odom;
  end_odom.z = z_in_odom;
  odom_arrow.points = {start_odom, end_odom};  // The list initialization syntax {} can be used to initialize a vector (obstacle_arrow.points) with a list of elements.
  
  odom_arrow.color.r = 0.0;
  odom_arrow.color.g = 0.0;
  odom_arrow.color.b = 1.0;
  odom_arrow.color.a = 1.0;

  odom_arrow.scale.x = 0.02;
  odom_arrow.scale.y = 0.1;
  odom_arrow.scale.z = 0.1;

  // Creating markers for obstacles detected in head_2_link frame
  visualization_msgs::msg::Marker head_2_link_arrow;
  head_2_link_arrow.header.frame_id = "head_2_link";
  head_2_link_arrow.header.stamp = now();
  head_2_link_arrow.type = visualization_msgs::msg::Marker::ARROW;
  head_2_link_arrow.action = visualization_msgs::msg::Marker::ADD;  // Adding the arrow marker to the visualization
  head_2_link_arrow.lifetime = rclcpp::Duration(1s);
  geometry_msgs::msg::Point start_head_2_link;
  start_head_2_link.x = 0.0;
  start_head_2_link.y = 0.0;
  start_head_2_link.z = 0.0;
  geometry_msgs::msg::Point end_head_2_link;
  end_head_2_link.x = x_in_head_2_link;
  end_head_2_link.y = y_in_head_2_link;
  end_head_2_link.z = z_in_head_2_link;
  head_2_link_arrow.points = {start_head_2_link, end_head_2_link};  // The list initialization syntax {} can be used to initialize a vector (obstacle_arrow.points) with a list of elements.
  
  head_2_link_arrow.color.r = 1.0;
  head_2_link_arrow.color.g = 0.0;
  head_2_link_arrow.color.b = 1.0;
  head_2_link_arrow.color.a = 1.0;

  head_2_link_arrow.scale.x = 0.02;
  head_2_link_arrow.scale.y = 0.1;
  head_2_link_arrow.scale.z = 0.1;

  marker_pub_->publish(obstacle_arrow);
  odom_marker_pub_->publish(odom_arrow);
  head_2_link_marker_pub_->publish(head_2_link_arrow);
}

}  // namespace br2_tf2_detector
