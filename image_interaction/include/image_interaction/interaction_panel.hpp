#ifndef IMAGE_INTERACTION__INTERACTION_PANEL_HPP_
#define IMAGE_INTERACTION__INTERACTION_PANEL_HPP_

#include <QImage>
#include <QLabel>
#include <QPixmap>

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "rviz_common/ros_topic_display.hpp"
#include "sensor_msgs/msg/image.hpp"

// #include "image_transport/image_transport.hpp"

// this class should support: MVP!!!
// 1. display images
// 2. a button to trigger segmentation request
// -------------------------------------------------
// 3. display segmentation results
// 4. marker display by class (position negative)

//                   response -->
// SAM server    <-- request         ImageInteraction (load plugin?? or hard depend on SAM for now)
// (labeled marker (pos-neg), bounding box)

namespace image_interaction {
class InteractionPanel : public rviz_common::Panel {
  // Q_OBJECT

public:
  InteractionPanel(QWidget *parent = nullptr) : Panel(parent){};

  void onInitialize() override {
    node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", 10,
        std::bind(&InteractionPanel::onImage, this, std::placeholders::_1));

    // image display
    label_ = std::make_unique<QLabel>(this);
    label_->setAlignment(Qt::AlignCenter);

    label_->setFixedSize(320, 240);
  }

public Q_SLOTS:
  void setWhatever(float whatever) {
    RCLCPP_INFO(node_->get_logger(), "setWhatever: %f", whatever);
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(node_->get_logger(), "got image: %d x %d", msg->width, msg->height);
    QImage img = QImage(msg->data.data(), msg->width, msg->height, QImage::Format_RGB888);
    label_->setPixmap(QPixmap::fromImage(img));
    label_->show();
  }

protected:
  std::unique_ptr<QLabel> label_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};
} // end of namespace image_interaction
#endif // IMAGE_INTERACTION__INTERACTION_PANEL_HPP_
