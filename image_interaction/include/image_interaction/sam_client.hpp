#ifndef image_interaction__SAM_CLIENT_HPP_
#define image_interaction__SAM_CLIENT_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "ros_sam_msgs/srv/segmentation.hpp"

namespace image_interaction {

class SamClient {
public:
  SamClient(const rclcpp::Node::SharedPtr node_ptr);
  ~SamClient() = default;

  ros_sam_msgs::srv::Segmentation_Response
  sync_segmentation_request(const ros_sam_msgs::srv::Segmentation_Request &request) const;

protected:
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Client<ros_sam_msgs::srv::Segmentation>::SharedPtr sam_client_ptr_;
};
} // end of namespace image_interaction
#endif // image_interaction__SAM_CLIENT_HPP_
