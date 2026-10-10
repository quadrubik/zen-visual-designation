import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

from geometry_msgs.msg import TransformStamped
from mavros_msgs.srv import MessageInterval
from mavros_msgs.msg import RCIn

# Package tiers : topics.py ne décrit que les topics /aeac écrits par l'équipe.
MAVROS_MESSAGE_INTERVAL = '/mavros/set_message_interval'
MAVROS_LOCAL_POSE = '/mavros/local_position/pose'
MAVROS_RC_IN = '/mavros/rc/in'

# Mavlink message ID
LOCAL_POSITION_NED = 32
RC_CHANNELS = 65

PWM_LOW, PWM_CENTER, PWM_HIGH = 1100, 1500, 1900

class GimbalSimulation(Node):

    def __init__(self):
        super().__init__('gimbal_simulation')

        self.set_up_parameters()
        self.set_up_topics()
        self.set_up_messages()
        self.set_up_timers()

        self.get_logger().info("Simulation de Gimbal démarrée.")


    def set_up_parameters(self):
        # Fréquence de publication de la transformation
        self.declare_parameter('publish_frequency', 20)
        self.publish_frequency = 1.0/self.get_parameter('publish_frequency').value

        # Rotation speed of the Gimbal
        self.rotation_speed = self.declare_parameter('rotation_speed', 0.01).value

        # Position relative du gimbal au drone
        self.x = self.declare_parameter('x', 2.0).value
        self.y = self.declare_parameter('y', 0.0).value
        self.z = self.declare_parameter('z', 2.0).value

        # Angle initials du gimbal
        self.yaw = self.declare_parameter('initial_yaw', 0.0).value
        self.roll = self.declare_parameter('initial_roll', 0.0).value
        self.pitch = self.declare_parameter('initial_pitch', 0.0).value

        # Channels RC de contrôle du gimbal
        self.rc_channel_x = self.declare_parameter('rc_channel_x', 5).value
        self.rc_channel_y = self.declare_parameter('rc_channel_y', 6).value

        self.rc_deadzone = self.declare_parameter('rc_deadzone', 50).value

        # Variables non paramétrées
        self.channels = []
        self.last_rc_received = None

    def set_up_topics(self):
        # Client to ask mavros for message initiation
        self.msg_interval_client = self.create_client(MessageInterval, MAVROS_MESSAGE_INTERVAL)
        while not self.msg_interval_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service non disponible, nouvelle tentative...')

        # Mavros local position subscriber
        #qos_profile_BE = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #                            history=QoSHistoryPolicy.KEEP_LAST,
        #                            depth=8)
        #self.pos_sub = self.create_subscription(PoseStamped, MAVROS_LOCAL_POSE, self.pos_callback, qos_profile_BE)

        self.create_subscription(RCIn, MAVROS_RC_IN, self.rc_callback, 10)

        # Broadcaster de la transformation "drone"->"gimbal"
        self.tf_broadcaster = TransformBroadcaster(self)
        
    def set_up_messages(self):
        #self.request_message(LOCAL_POSITION_NED, 10.0)
        self.request_message(RC_CHANNELS, 10.0)

    def request_message(self, message_id, message_rate):
        #self.get_logger().info(f"Message demandé : {message_id}")
        request = MessageInterval.Request()
        request.message_id = message_id
        request.message_rate = message_rate

        future = self.msg_interval_client.call_async(request)
        future.add_done_callback(self.message_interval_callback)

    def message_interval_callback(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info(f"Message accepté : {response}")
        else:
            self.get_logger().warn(f"Message refusé (result={response.result})")

    def set_up_timers(self):
        self.update_gimbal_timer = self.create_timer(0.05, self.update)
        self.broadcast_timer = self.create_timer(self.publish_frequency, self.publish_broadcast)

    def update(self):
        # Check if no rc input received in the last second
        if (self.last_rc_received is None or
            (self.get_clock().now() - self.last_rc_received).nanoseconds > 1e9):
            return
        
        x_value = self.channels[self.rc_channel_x]
        y_value = self.channels[self.rc_channel_y]

        x_value = x_value - PWM_CENTER
        y_value = y_value - PWM_CENTER

        if (abs(x_value) <= self.rc_deadzone):
            x_value = 0

        if (abs(y_value) <= self.rc_deadzone):
            y_value = 0

        x_value = self.rotation_speed * x_value * 2/(PWM_HIGH-PWM_LOW)
        y_value = self.rotation_speed * y_value * 2/(PWM_HIGH-PWM_LOW)

        self.yaw   += x_value
        self.pitch += y_value

    def publish_broadcast(self):
        tf = TransformStamped()

        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'drone'
        tf.child_frame_id = 'gimbal'

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = self.z
        quat = quaternion_from_euler(
            float(self.roll), float(self.pitch), float(self.yaw))
        tf.transform.rotation.x = quat[0]
        tf.transform.rotation.y = quat[1]
        tf.transform.rotation.z = quat[2]
        tf.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(tf)

    def rc_callback(self, msg):
        self.last_rc_received = self.get_clock().now()
        self.channels = msg.channels


def main(args=None):
    rclpy.init(args=args)
    node = GimbalSimulation()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()