#ifndef IMAGE_INTERACTION__INTERACTION_PANEL_HPP_
#define IMAGE_INTERACTION__INTERACTION_PANEL_HPP_

#include <QCloseEvent>
#include <QEvent>
#include <QFocusEvent>
#include <QImage>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMoveEvent>
#include <QPixmap>
#include <QPushButton>
#include <QResizeEvent>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QWidget>

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

// consider using rviz image texture built on ogre
// https://github.com/ros2/rviz/blob/rolling/rviz_default_plugins/src/rviz_default_plugins/displays/image/ros_image_texture.cpp
// require:
//  - topic name
//  - image stack
//  - tools (paint / annotate)
//  - clear
//  - segmentation request
//  - segmentation result display

namespace image_interaction {
class InteractiveImageWidget : public QWidget {
public:
  InteractiveImageWidget(
      QWidget *parent = nullptr,
      rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface = nullptr)
      : QWidget(parent), logging_interface_(logging_interface) {
    auto label = new QLabel(this); // on click must be inside label
    label->setAlignment(Qt::AlignCenter);
    label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    label->setFixedSize(320, 240);

    image_buffer_.fill(Qt::white);
    label->setPixmap(QPixmap::fromImage(image_buffer_));
    label->show();
  };

protected:
  void mousePressEvent(QMouseEvent *event) override {
    RCLCPP_INFO(logging_interface_->get_logger(), "mouse press: %d, %d", event->x(), event->y());
  }

  void mouseMoveEvent(QMouseEvent *event) override {
    RCLCPP_INFO(logging_interface_->get_logger(), "mouse move: %d, %d", event->x(), event->y());
  }

  void mouseReleaseEvent(QMouseEvent *event) override {
    RCLCPP_INFO(logging_interface_->get_logger(), "mouse release: %d, %d", event->x(), event->y());
  }

  void wheelEvent(QWheelEvent *event) override {
    RCLCPP_INFO(logging_interface_->get_logger(), "wheel: %d", event->angleDelta().y());
  }

  void keyPressEvent(QKeyEvent *event) override {
    RCLCPP_INFO(logging_interface_->get_logger(), "key press: %d", event->key());
  }

  void keyReleaseEvent(QKeyEvent *event) override {
    RCLCPP_INFO(logging_interface_->get_logger(), "key release: %d", event->key());
  }

  void focusInEvent(QFocusEvent *event) override {
    RCLCPP_INFO(logging_interface_->get_logger(), "focus in");
  }

  void focusOutEvent(QFocusEvent *event) override {
    RCLCPP_INFO(logging_interface_->get_logger(), "focus out");
  }

  void enterEvent(QEvent *event) override {
    RCLCPP_INFO(logging_interface_->get_logger(), "enter");
  }

  void leaveEvent(QEvent *event) override {
    RCLCPP_INFO(logging_interface_->get_logger(), "leave");
  }

  void resizeEvent(QResizeEvent *event) override {
    height_ = event->size().height();
    width_ = event->size().width();
    RCLCPP_INFO(logging_interface_->get_logger(), "resize: %d x %d", width_, height_);
    image_buffer_ = image_buffer_.scaled(width_, height_, Qt::KeepAspectRatio);
    auto label = findChild<QLabel *>();
    label->setPixmap(QPixmap::fromImage(image_buffer_));
    label->show();
  }

  void moveEvent(QMoveEvent *event) override {
    RCLCPP_INFO(logging_interface_->get_logger(), "move: %d, %d", event->pos().x(),
                event->pos().y());
  }

  void closeEvent(QCloseEvent *event) override {
    RCLCPP_INFO(logging_interface_->get_logger(), "close");
  }

public:
  void setImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(logging_interface_->get_logger(), "updating image");
    QImage img = QImage(msg->data.data(), msg->width, msg->height, QImage::Format_RGB888);
    img = img.scaled(width_, height_, Qt::KeepAspectRatio);
    image_buffer_ = img;
    auto label = findChild<QLabel *>();
    label->setPixmap(QPixmap::fromImage(image_buffer_));
    label->show();
  }

protected:
  int height_, width_;

  QImage image_buffer_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_;
};

class InteractionPanel : public rviz_common::Panel {
  // Q_OBJECT

public:
  InteractionPanel(QWidget *parent = nullptr) : Panel(parent){};

  void onInitialize() override {
    node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    //
    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", 10,
        std::bind(&InteractionPanel::onImage, this, std::placeholders::_1));

    // image display

    // label_ = std::make_unique<QLabel>();
    // label_->setAlignment(Qt::AlignCenter);
    // label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto interactive_image_widget =
        new InteractiveImageWidget(this, node_->get_node_logging_interface());

    // label_->setFixedSize(320, 240);

    // add editable text
    auto text = new QLineEdit("Hello", this);
    text->setPlaceholderText("Enter your name");
    text->setAlignment(Qt::AlignCenter);

    // add button
    auto button = new QPushButton("Annotate", this);
    connect(button, &QPushButton::clicked, this, &InteractionPanel::setWhatever);

    // layout
    auto layout = new QVBoxLayout;
    layout->addWidget(interactive_image_widget);
    layout->addWidget(button);
    layout->addWidget(text);
    setLayout(layout);
  }

public:
  void setWhatever(float whatever) {
    RCLCPP_INFO(node_->get_logger(), "setWhatever: %f", whatever);
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    findChild<InteractiveImageWidget *>()->setImage(msg);
    RCLCPP_INFO(node_->get_logger(), "got image: %d x %d", msg->width, msg->height);
    // QImage img = QImage(msg->data.data(), msg->width, msg->height, QImage::Format_RGB888);
    // // buffer image and scale to fit the label on label signal
    // img = img.scaled(label_->width(), label_->height(), Qt::IgnoreAspectRatio);
    // label_->setPixmap(QPixmap::fromImage(img));
    // label_->show();
  }

protected:
  std::unique_ptr<QLabel> label_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};
} // end of namespace image_interaction
#endif // IMAGE_INTERACTION__INTERACTION_PANEL_HPP_
