#ifndef RVIZ2_SAM_PLUGIN__SAM_PLUGIN_HPP_
#define RVIZ2_SAM_PLUGIN__SAM_PLUGIN_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "ros_sam_msgs/srv/segmentation.hpp"

#include "rviz2_sam_plugin/sam_client.hpp"

namespace rviz2_sam_plugin {
class SamPlugin : public rviz::Panel {
public:
  //   SamPlugin(QWidget *parent = nullptr);

  //   void onInitialize() override;

  //   void onSegmentationButtonClicked();

  //   void onImageReceived(const sensor_msgs::msg::Image::SharedPtr msg);

protected:
  rclcpp::Node::SharedPtr node_ptr_;
  std::shared_ptr<rviz2_sam_plugin::SamClient> sam_client_;
};
} // end of namespace rviz2_sam_plugin
#endif // RVIZ2_SAM_PLUGIN__SAM_PLUGIN_HPP_