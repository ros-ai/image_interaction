#ifndef IMAGE_INTERACTION__INTERACTION_PANEL_HPP_
#define IMAGE_INTERACTION__INTERACTION_PANEL_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace image_interaction {
class InteractionPanel : public rviz_common::Panel {
public:
  InteractionPanel(QWidget *parent = nullptr){};

  void onInitialize() override {};
  void onSegmentationButtonClicked() {};
  void onImageReceived(const sensor_msgs::msg::Image::SharedPtr msg) {};

protected:
  rclcpp::Node::SharedPtr node_ptr_;
};
} // end of namespace image_interaction
#endif // IMAGE_INTERACTION__INTERACTION_PANEL_HPP_
