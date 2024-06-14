import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt

class MQTTSubscriber(Node):

    def __init__(self):
        super().__init__('mqtt_subscriber')
        self.subscription = self.create_subscription(
            String,
            'mqtt_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # MQTT client setup
        self.mqtt_client = mqtt.Client("UbuntuSubscriber")
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.mqtt_client.connect("localhost", 1883, 60)  # Using localhost for ROS 2 topic
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT broker")
        client.subscribe("test/topic")

    def on_message(self, client, userdata, msg):
        self.get_logger().info(f"Received message: {msg.payload.decode()}")
        ros_msg = String()
        ros_msg.data = msg.payload.decode()
        self.publisher.publish(ros_msg)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    mqtt_subscriber = MQTTSubscriber()

    rclpy.spin(mqtt_subscriber)

    mqtt_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
