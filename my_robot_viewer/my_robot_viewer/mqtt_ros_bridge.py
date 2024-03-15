import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import paho.mqtt.client as mqtt
import yaml
import cv2
import numpy as np
import base64

class MqttRosBridge(Node):
    def __init__(self, mqtt_config_path):
        super().__init__('mqtt_ros_bridge')

        # Load MQTT configuration from YAML file
        with open(mqtt_config_path, 'r') as file:
            mqtt_config = yaml.safe_load(file)

        # Extract MQTT broker information from the loaded configuration
        mqtt_broker_address = mqtt_config['broker']['host']
        mqtt_broker_port = mqtt_config['broker']['port']
        mqtt_username = mqtt_config['broker']['username']
        mqtt_password = mqtt_config['broker']['password']
        mqtt_topic = "mqtt_video" 

        # ROS publisher to publish received MQTT messages
        self.publisher_ = self.create_publisher(Image, 'mqtt_ros_topic', 10)
        self.cv_bridge = CvBridge()

        # MQTT client setup
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_disconnect = self.on_disconnect
        self.mqtt_client.on_message = self.on_message

        self.mqtt_client.username_pw_set(mqtt_username, mqtt_password)
        self.mqtt_client.connect(mqtt_broker_address, mqtt_broker_port, 60)
        self.mqtt_client.subscribe(mqtt_topic)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected from MQTT broker with result code %s" % rc)
            print("Connected to MQTT broker with result code %s" % rc)
        else:
            print("connection failed")

    def on_disconnect(self, client, userdata, rc):
        self.get_logger().info("Disconnected from MQTT broker with result code %s" % rc)

    def on_message(self, client, userdata, msg):
        if msg:
            print(msg.topic)
            try:
                # Assuming the received MQTT message is the image data in bytes
                self.get_logger().info(f"receiveing video frame from MQTT")
                print(f"message payload: {msg.payload.decode()}")
                print(msg)
                print(msg.payload)
                img = base64.b64decode(msg.payload)
                image_data = np.frombuffer(img, dtype=np.uint8)
                cv_image = cv2.imdecode(image_data, 1)

                # Create a ROS 2 Image message
                ros_image = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

                # Publish the ROS 2 Image message to a ROS 2 topic
                self.publisher_.publish(ros_image)
                self.get_logger().info("Published video frame to ROS 2 topic")

            except Exception as e:
                self.get_logger().error(f"Error processing MQTT message: {str(e)}")
        else:
            print("not receiving any messages")

def main(args=None):
    rclpy.init(args=args)

    mqtt_config_path = '/home/nikhil/Desktop/mqtt_ws/src/my_robot_viewer/config/mqtt_ros_config.yaml'

    node = MqttRosBridge(mqtt_config_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()