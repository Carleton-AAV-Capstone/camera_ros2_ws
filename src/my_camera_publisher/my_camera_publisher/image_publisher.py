import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/left/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info("📡 Subscribed to /left/image_raw")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Left Camera View", cv_image)
            cv2.waitKey(1)
            self.get_logger().info("🖼️ Received image from /left/image_raw")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to convert image: {e}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
