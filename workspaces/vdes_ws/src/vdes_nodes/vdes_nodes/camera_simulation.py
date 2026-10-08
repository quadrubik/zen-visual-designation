from vdes_interfaces.msg import PointList

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.srv import MessageInterval

class CameraSimulation(Node):

    def __init__(self):
        super().__init__('camera_simulation')

        self.set_up_parameters()
        self.set_up_topics()
        self.set_up_messages()
        self.set_up_timers()

        self.get_logger().info("Simulation de caméra démarrée.")

    def set_up_parameters(self):
        # Focal length of the camera
        self.declare_parameter('focal_length', 2.0)
        self.focal_length = self.get_parameter('focal_length').value

    def set_up_topics(self):
        # Get information about targets positions
        self.target_sub = self.create_subscription(PointList, 'get_targets', self.camera_timer_callback, 10)

        # Client to ask mavros for message initiation
        self.msg_interval_client = self.create_client(MessageInterval, '/mavros/set_message_interval')
        while not self.msg_interval_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service non disponible, nouvelle tentative...')

        # Mavros local position subscriber
        qos_profile_BE = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                            history=QoSHistoryPolicy.KEEP_LAST,
                                            depth=8)
        self.pos_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pos_callback, qos_profile_BE)

    def set_up_messages(self):
        self.request_message(32, 10.0)

    def request_message(self, message_id, message_rate):
        request = MessageInterval.Request()
        request.message_id = message_id
        request.message_rate = message_rate

        future = self.msg_interval_client.call_async(request)
        future.add_done_callback(self.message_interval_callback)

    def message_interval_callback(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info(f"Message demandé : {response}")
        else:
            self.get_logger().warn(f"Message refusé (result={response.result})")

    def set_up_timers(self):
        #self.camera_time = self.create_timer(0.2, self.camera_timer_callback)
        pass

    def camera_timer_callback(self, msg):
        self.get_logger().info(f"Received targets : {msg}")

    def pos_callback(self, msg):
        self.drone_pos = msg

        # Calculate rotation matrixes
        

def main(args=None):
    rclpy.init(args=args)
    camera_simulation = CameraSimulation()
    try:
        rclpy.spin(camera_simulation)
    except KeyboardInterrupt:
        pass
    finally:
        camera_simulation.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=="__main__":
    main()