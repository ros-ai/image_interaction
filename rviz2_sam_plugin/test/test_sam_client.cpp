#include <filesystem>
#include <gtest/gtest.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rviz2_sam_plugin/sam_client.hpp"
#include "ros_sam_msgs/srv/segmentation.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui.hpp"

class SamClientTestFixture : public ::testing::Test {
public:
  SamClientTestFixture() : node_(rclcpp::Node::make_shared("sam_client_test")) {
    sam_client_ = std::make_shared<rviz2_sam_plugin::SamClient>(node_);

    test_image_ =
        cv::imread(std::filesystem::path(ament_index_cpp::get_package_share_directory("ros_sam")) /
                       "data" / "car.jpg",
                   cv::IMREAD_COLOR);
  }

protected:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rviz2_sam_plugin::SamClient> sam_client_;

  // test cv2 image
  cv::Mat test_image_;
};

TEST_F(SamClientTestFixture, test_sync_segmentation_request) {
  ros_sam_msgs::srv::Segmentation_Request request;
  sam_client_->sync_segmentation_request(request);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
