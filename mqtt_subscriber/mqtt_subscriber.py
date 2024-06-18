import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt

class MQTTSubscriber(Node):

    def __init__(self):
        super().__init__('mqtt_subscriber')
        self.publisher = self.create_publisher(String, 'mqtt_topic', 10)
        self.subscription = self.create_subscription(
            String,
            'mqtt_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # MQTT client setup
        self.mqtt_client = mqtt.Client("UbuntuSubscriber", userdata=None, protocol=mqtt.MQTTv311, transport="tcp", callback_api_version=5)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # Update the broker address to the correct IP
        self.mqtt_client.connect("192.168.0.12", 1883, 60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT broker")
            client.subscribe("test/topic")
        else:
            self.get_logger().error(f"Connection failed with code {rc}")

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
