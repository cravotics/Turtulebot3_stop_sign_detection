import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.stop_sign_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'stop_sign_classifier_2.xml')

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        stop_signs = self.stop_sign_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        # Movement command
        move_cmd = Twist()

        if len(stop_signs) > 0:
            # Stop the robot if a stop sign is detected
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.get_logger().info('Stop sign detected. Stopping robot.')
        else:
            # Move forward if no stop sign is detected
            move_cmd.linear.x = 0.2  # Speed value can be adjusted
            move_cmd.angular.z = 0.0

        # Publish the movement command
        self.publisher.publish(move_cmd)

        # Show detection on screen
        for (x, y, w, h) in stop_signs:
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 0, 255), 2)
        cv2.imshow("Stop Sign Detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetector()
    rclpy.spin(object_detector)
    object_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
