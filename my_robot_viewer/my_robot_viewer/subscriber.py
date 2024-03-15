import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
 
class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sub = self.create_subscription(
            Image, 'mqtt_ros_topic', self.listener_callback, 10)
        self.cv_bridge = CvBridge()
 
    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        cv2.imshow("subscriber", image)
        cv2.waitKey(1)
 
def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber("webcam_sub")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
