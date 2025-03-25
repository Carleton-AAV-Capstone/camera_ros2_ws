import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.publish_image)  # 10 FPS
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # Open default camera

        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera!")

        self.get_logger().info("Image Publisher Node Started!")

    def publish_image(self):
        ret, frame = self.cap.read()
        if ret:
            try:
                # Convert OpenCV image to ROS2 Image message
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.publisher.publish(msg)
                self.get_logger().info("Published Image Frame")
            except Exception as e:
                self.get_logger().error(f"Failed to publish image: {e}")
        else:
            self.get_logger().error("Failed to capture image from camera!")

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

