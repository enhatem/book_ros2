#ifndef BR2_TF2_DETECTOR__MOVEMENTMONITORNODE_HPP_
#define BR2_TF2_DETECTOR__MOVEMENTMONITORNODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// #include "geometry_msgs/msg/transform_stamped.hpp"


namespace br2_tf2_detector{

class MovementMonitorNode : public rclcpp::Node {
public:
    MovementMonitorNode();

private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;

    tf2::BufferCore tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::msg::TransformStamped odom2base_footprint_previous;  // at time t-1
    bool transform_data_stored;
};

} // br2_tf2_detector

#endif