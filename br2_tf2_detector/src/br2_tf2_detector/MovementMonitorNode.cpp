#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <br2_tf2_detector/MovementMonitorNode.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>

namespace br2_tf2_detector
{

using namespace std::chrono_literals;

MovementMonitorNode::MovementMonitorNode()
: Node("movement_monitor"),
tf_buffer_(),
tf_listener_(tf_buffer_), transform_data_stored(false)
{
    timer_ = create_wall_timer(1s, std::bind(&MovementMonitorNode::timer_callback, this));
}

void
MovementMonitorNode::timer_callback()
{
    geometry_msgs::msg::TransformStamped odom2base_footprint_current;
    try{
        odom2base_footprint_current = tf_buffer_.lookupTransform(
            "odom", "base_footprint", tf2::TimePointZero);  // tf2::TimePointZero is used to get the latest available transform in the buffer
    } catch (tf2::TransformException & ex){
        RCLCPP_WARN(get_logger(), "Requested transform not found: %s", ex.what());
        return;
    }
    
    if (transform_data_stored == false){
    odom2base_footprint_previous = odom2base_footprint_current;
    transform_data_stored = true;
    return;
    }

    // Performing calculations to find the distance travelled
    double x_dif = odom2base_footprint_current.transform.translation.x - odom2base_footprint_previous.transform.translation.x;
    double y_dif = odom2base_footprint_current.transform.translation.y - odom2base_footprint_previous.transform.translation.y;

    double dist = sqrt( pow(x_dif, 2) + pow(y_dif, 2) );

    // Printing distance travelled on console
    RCLCPP_INFO(get_logger(), "Distance travalled since 1s: %lf", dist);

    // Replacing previous value of odom2base_foorprint with current one
    odom2base_footprint_previous = odom2base_footprint_current;

}

}// namespace br2_tf2_detector

