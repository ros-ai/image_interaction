import cv2
import cv_bridge
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node


class StrawberryPub(Node):
    def __init__(self, image_path: str) -> None:
        super().__init__("strawberry_pub")
        self.publisher = self.create_publisher(Image, "/camera/color/image_raw", 1)

        self.timer = self.create_timer(1.0, self.publish)

        self.bridge = cv_bridge.CvBridge()
        self.image = cv2.imread(image_path)
        self.image_msg = self.bridge.cv2_to_imgmsg(self.image, encoding="passthrough")

    def publish(self):
        self.get_logger().info("Image published")
        self.publisher.publish(self.image_msg)

    def destroy_node(self):
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    strawberry_pub = StrawberryPub("strawberries.jpg")
    rclpy.spin(strawberry_pub)


if __name__ == "__main__":
    main()
