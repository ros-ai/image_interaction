#include "image_interaction/sam_client.hpp"

namespace image_interaction {
SamClient::SamClient(const rclcpp::Node::SharedPtr node_ptr) : node_ptr_(node_ptr) {
  sam_client_ptr_ =
      node_ptr_->create_client<ros_sam_msgs::srv::Segmentation>("/sam_server/segment");

  while (!sam_client_ptr_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Waiting for %s service to become available...",
                sam_client_ptr_->get_service_name());
  }
  RCLCPP_INFO(node_ptr_->get_logger(), "Established client to %s service.",
              sam_client_ptr_->get_service_name());
}

ros_sam_msgs::srv::Segmentation_Response
SamClient::sync_segmentation_request(const ros_sam_msgs::srv::Segmentation_Request &request) const {
  //   auto request = std::make_shared<ros_sam_msgs::srv::Segmentation::Request>();
  //   request->input_image = sensor_msgs::msg::Image();
  //   request->input_image.header.frame_id = "camera_link";
  //   request->input_image.height = 480;
  //   request->input_image.width = 640;
  //   request->input_image.encoding = "rgb8";
  //   request->input_image.is_bigendian = false;
  //   request->input_image.step = 640 * 3;
  //   request->input_image.data =
  //       std::vector<uint8_t>(request->input_image.step * request->input_image.height, 0);

  //   auto result = sam_client_ptr_->async_send_request(request);
  //   if (rclcpp::spin_until_future_complete(node_ptr_, result) !=
  //       rclcpp::executor::FutureReturnCode::SUCCESS) {
  //     RCLCPP_ERROR(node_ptr_->get_logger(), "Failed to call service %s",
  //                  sam_client_ptr_->get_service_name());
  //     return false;
  //   }
  //   RCLCPP_INFO(node_ptr_->get_logger(), "Service %s returned %d",
  //               sam_client_ptr_->get_service_name(), result.get()->output_image.height);
}
} // end of namespace image_interaction
