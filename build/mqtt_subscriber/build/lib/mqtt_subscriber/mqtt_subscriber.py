import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt

class MQTTSubscriber(Node):

    def __init__(self):
        super().__init__('mqtt_subscriber')

        # Create publisher
        #self.coord_publisher = self.create_publisher(String, 'coordination_list', 10)
        # self.timer = self.create_timer(1.0, self.read_from_serial)

        self.publisher = self.create_publisher(String, 'coordination_list', 10)
        self.subscription = self.create_subscription(
            String,
            'coordination_list',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # MQTT client setup
        self.mqtt_client = mqtt.Client("UbuntuSubscriber", userdata=None, protocol=mqtt.MQTTv311, transport="tcp")
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # Connect to the Windows PC MQTT broker
        self.mqtt_client.connect("192.168.0.12", 1883, 60)  # Replace with your Windows PC IP address
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
