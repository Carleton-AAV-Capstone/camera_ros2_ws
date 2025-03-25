import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ultralytics import YOLO

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Topic from camera publisher
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Image, '/yolo/detections', 10)
        self.bridge = CvBridge()

        # Load YOLO model using a portable path
        model_path = os.path.expanduser("~/Yolo/Capstone/runs/detect/train7/weights/best.pt")
        if not os.path.exists(model_path):
            self.get_logger().error(f"YOLO weights file not found: {model_path}")
            raise FileNotFoundError(f"YOLO weights file not found: {model_path}")

        self.model = YOLO(model_path)
        self.get_logger().info("YOLO Detector Node Started!")

    def image_callback(self, msg):
        try:
            # Convert ROS2 Image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Run YOLO inference with error handling
            try:
                results = self.model(frame)
                if results:
                    annotated_frame = results[0].plot()
                else:
                    self.get_logger().warn("YOLO returned empty results.")
                    return
            except Exception as e:
                self.get_logger().error(f"YOLO inference failed: {e}")
                return

            # Convert OpenCV image back to ROS2 Image and publish
            ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.publisher.publish(ros_image)

            # Display annotated frame
            cv2.imshow("YOLO Detections", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down YOLO node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()  # Cleanup OpenCV windows

if __name__ == '__main__':
    main()

