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

#include <vector>

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "br2_tracking/ObjectDetector.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

namespace br2_tracking
{

using std::placeholders::_1;

ObjectDetector::ObjectDetector()
: Node("object_detector")
{
  image_sub_ = image_transport::create_subscription(
    this, "input_image", std::bind(&ObjectDetector::image_callback, this, _1),
    "raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());

  detection_pub_ = create_publisher<vision_msgs::msg::Detection2D>("detection", 100);

  declare_parameter("hsv_ranges", hsv_filter_ranges_);
  declare_parameter("debug", debug_);

  get_parameter("hsv_ranges", hsv_filter_ranges_);
  get_parameter("debug", debug_);
}

void
ObjectDetector::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (detection_pub_->get_subscription_count() == 0) {return;}

  const float & h = hsv_filter_ranges_[0];
  const float & H = hsv_filter_ranges_[1];
  const float & s = hsv_filter_ranges_[2];
  const float & S = hsv_filter_ranges_[3];
  const float & v = hsv_filter_ranges_[4];
  const float & V = hsv_filter_ranges_[5];

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  // converting received image to CvImage format with BGR8 encoding
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat img_hsv;
  cv::cvtColor(cv_ptr->image, img_hsv, cv::COLOR_BGR2HSV);  // converting BGR color space to the HSV color space

  cv::Mat1b filtered;  // represents a matrix with a single channel of 8-bit unsigned integers ==> this data type is used since the output of cv::inRange will be a binary image, which has a single channel and pixel values of either 0 or 255.
  cv::inRange(img_hsv, cv::Scalar(h, s, v), cv::Scalar(H, S, V), filtered);

  // applying noise filtering to the image
  cv::medianBlur(filtered, filtered, 17);

  // finding contours in the mask
  std::vector<std::vector<cv::Point>> contours;  // TODO: Check what are elements hierarchies
  cv::findContours(filtered, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  // checking if any contours were found 
  if (contours.empty()) {
    RCLCPP_INFO(this->get_logger(), "No contours found.");
    return;
  }
  // calculating moments of first detected contour
  cv::Moments first_moment = cv::moments(contours[0], false);  // there's a bug here
  // skipping iteration if no object detected
  if (first_moment.m00 < 1) {return;}
  // calculating the center of mass of the first detected contour
  cv::Point2f centeroid = cv::Point2f(first_moment.m10 / first_moment.m00, first_moment.m01 / first_moment.m00);
  auto cx = centeroid.x;
  auto cy = centeroid.y;
  // calculating the bounding box of the first detected contour
  cv::Rect bbx = cv::boundingRect(contours[0]);

  // auto moment = cv::moments(filtered, true);
  // cv::Rect bbx = cv::boundingRect(filtered);

  // auto m = cv::moments(filtered, true);  // calculating the moments of the filtered image 
  // if (m.m00 < 0.000001) {return;}  // return if the area of the image is less than 0.000001 ==> no object detected in binary image (black image without any white pixels)
  // calculating the center of mass (centroid) coordinates
  // int cx = m.m10 / m.m00;
  // int cy = m.m01 / m.m00;

  vision_msgs::msg::Detection2D detection_msg;
  detection_msg.header = msg->header;
  detection_msg.bbox.size_x = bbx.width;
  detection_msg.bbox.size_y = bbx.height;
  detection_msg.bbox.center.x = cx;
  detection_msg.bbox.center.y = cy;
  detection_msg.source_img = *cv_ptr->toImageMsg();
  detection_pub_->publish(detection_msg);

  if (debug_) {
    // drawing bounding boxes for all contours
    for (const auto & contour : contours) {
      cv::Rect bounding_rect = cv::boundingRect(contour);
      cv::rectangle(cv_ptr->image, bounding_rect, cv::Scalar(0, 0, 255), 2);
    }
    // cv::rectangle(cv_ptr->image, bbx, cv::Scalar(0, 0, 255), 3);  // Draws a rectangle on the image cv_ptr->image using the bounding box bbx with a red color (BGR format) and a line thickness of 3 pixels.
    cv::circle(cv_ptr->image, cv::Point(cx, cy), 3, cv::Scalar(255, 0, 0), 3);  // Draws a circle on the image cv_ptr->image at the center point (cx, cy) with a blue color (BGR format), a radius of 3 pixels, and a line thickness of 3 pixels.
    cv::imshow("cv_ptr->image", cv_ptr->image);
    cv::waitKey(1);
  }
}

}  // namespace br2_tracking
