#ifndef IMAGE_INTERACTION__INTERACTION_PANEL_HPP_
#define IMAGE_INTERACTION__INTERACTION_PANEL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "rviz_common/ros_topic_display.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "image_interaction/types.hpp"
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

// consider an rqt plugin instead written in python

// consider using rviz image texture built on ogre
// https://github.com/ros2/rviz/blob/rolling/rviz_default_plugins/src/rviz_default_plugins/displays/image/ros_image_texture.cpp
// require:
//  - topic name
//  - image stack
//  - tools (paint / annotate)
//  - clear
//  - segmentation request
//  - segmentation result display
// left click: positive samples
// right click: negative samples
// class point map

namespace image_interaction {

class InteractionPanel : public rviz_common::Panel {
  // Q_OBJECT

public:
  InteractionPanel(QWidget *parent = nullptr) : Panel(parent){};

  void onInitialize() override {
    node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  }

public:
protected:
  rclcpp::Node::SharedPtr node_;
};
} // end of namespace image_interaction
#endif // IMAGE_INTERACTION__INTERACTION_PANEL_HPP_
