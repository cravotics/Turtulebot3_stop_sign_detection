import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory


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
        # Set the absolute path to the classifier
        package_share_directory = get_package_share_directory('turtlebot3_stop_sign')
        cascade_path = os.path.join(package_share_directory, 'stop_sign_classifier_2.xml')
        self.stop_sign_cascade = cv2.CascadeClassifier(cascade_path)
        if self.stop_sign_cascade.empty():
            self.get_logger().error('Failed to load cascade classifier from {}'.format(cascade_path))
            raise Exception('Failed to load cascade classifier')

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        stop_signs = self.stop_sign_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        # Movement command
        move_cmd = Twist()
        if len(stop_signs) > 0:
            move_cmd.linear.x = 0.0  # Stop the robot
            self.get_logger().info('Stop sign detected. Stopping robot.')
        else:
            move_cmd.linear.x = 0.2  # Move the robot forward

        self.publisher.publish(move_cmd)
        

def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetector()
    rclpy.spin(object_detector)
    object_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
