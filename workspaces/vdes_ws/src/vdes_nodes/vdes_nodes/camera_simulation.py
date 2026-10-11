from vdes_interfaces.msg import PointList, FrameView, PointObservation

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import tf2_ros
from tf2_ros import TransformBroadcaster

import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from mavros_msgs.srv import MessageInterval
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType, Parameter as ParamMsg

# Pour deboger, à enlever
from tf_transformations import euler_from_quaternion
import math

# Package tiers : topics.py ne décrit que les topics /aeac écrits par l'équipe.
MAVROS_LOCAL_POSITION_PARAM = '/mavros/local_position/set_parameters'
MAVROS_MESSAGE_INTERVAL = '/mavros/set_message_interval'
MAVROS_LOCAL_POSE = '/mavros/local_position/pose'

# Mavlink message ID
ATTITUDE_QUATERNION = 31
LOCAL_POSITION_NED = 32

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
        self.focal_length = self.declare_parameter('focal_length', 2.0).value

        # Camera resolution
        self.image_width = self.declare_parameter('image_width', 1920).value
        self.image_height = self.declare_parameter('image_height', 1080).value

        # Camera horizontal FOV
        self.horizontal_fov = self.declare_parameter('horizontal_fov', 1.0472).value

        self.sensor_width = 2*self.focal_length*np.tan(self.horizontal_fov/2)
        self.sensor_height = self.image_height*self.sensor_width/self.image_width

        self.horizontal_pixel_density = self.image_width/self.sensor_width
        self.vertical_pixel_density = self.image_height/self.sensor_height

        self.focal_pixel_x = self.horizontal_pixel_density*self.focal_length
        self.focal_pixel_y = self.vertical_pixel_density*self.focal_length

        self.K = np.array([[self.focal_pixel_x, 0, self.image_width/2],
                           [0, self.focal_pixel_y, self.image_height/2],
                           [0, 0, 1]])


        # Changement du paramètre mavros pour l'envoie de frame tf
        self.cli = self.create_client(SetParameters, MAVROS_LOCAL_POSITION_PARAM)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {MAVROS_LOCAL_POSITION_PARAM}...')

        req = SetParameters.Request()
        p = ParamMsg()
        p.name = 'tf.send'
        p.value.type = ParameterType.PARAMETER_BOOL
        p.value.bool_value = True
        req.parameters = [p]

        future = self.cli.call_async(req)
        future.add_done_callback(self.set_parameters_callback)

    def set_parameters_callback(self, future):
        response = future.result().results
        for result in response :
            if not result.successful:
                self.get_logger().info(f"Message refusé (result={result.reason})")

    def set_up_topics(self):
        # Get information about targets positions
        self.target_sub = self.create_subscription(PointList, 'get_targets', self.target_callback, 10)
        self.targets = None

        # Client to ask mavros for message initiation
        self.msg_interval_client = self.create_client(MessageInterval, MAVROS_MESSAGE_INTERVAL)
        while not self.msg_interval_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service non disponible, nouvelle tentative...')

        # Transform buffer and listener, not sure it should be here
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # FrameView publisher
        self.frame_pub = self.create_publisher(FrameView, 'frame', 10)

    def target_callback(self, msg):
        #self.get_logger().info(f"Received targets : {msg}")
        self.targets = msg.points
        self.last_received_targets = self.get_clock().now()

    def set_up_messages(self):
        self.request_message(LOCAL_POSITION_NED, 20.0)
        self.request_message(ATTITUDE_QUATERNION, 20.0)

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
        self.camera_timer = self.create_timer(0.2, self.camera_timer_callback)

    def camera_timer_callback(self):
        # Should check target and position time coherence
        #self.get_logger().info(self.targets.points)
        #self.print_map_to_gimbal_angles()
    
        if (self.targets is None):
            return

        msg = FrameView()
        count = 1
        
        for target in self.targets:

            points_in = PointStamped()
            points_in.header.frame_id = "map"
            points_in.header.stamp = self.last_received_targets.to_msg()
            points_in.point = target

            try:
                points_out = self.tf_buffer.transform(
                    points_in,
                    "gimbal",
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )

                p = points_out.point
                # Convert from FLU to RDF
                points_out = np.array([[-p.y], [-p.z], [p.x]])

                #self.get_logger().info(f"Target in camera frame: {round(points_out[0, 0],2)}, {round(points_out[1, 0],2)}, {round(points_out[2, 0],2)}")

                if points_out[2, 0] > 0:
                    pixel_homog = self.K @ points_out
                    pixel_homog /= pixel_homog[2, 0]

                    u, v = int(pixel_homog[0, 0]), int(pixel_homog[1, 0])

                    if (0 < u < self.image_width and 0 < v < self.image_height):
                        self.get_logger().info(f"Target numéro {count} : (u, v) = ({u}, {v})")

                        po = PointObservation()
                        po.point = target
                        po.u, po.v = u, v
                        po.index = count

                        msg.points.append(po)

                        pass

            except tf2_ros.ExtrapolationException as e:
                self.get_logger().warn(f"TF extrapolation issue, skipping point: {e}")
                continue

            count += 1

        self.frame_pub.publish(msg)

    # Claude generated, debugging purpose.
    def print_map_to_gimbal_angles(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "map",
                "gimbal",
                rclpy.time.Time(),  # dernière transform disponible
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            q = t.transform.rotation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            self.get_logger().info(
                f"map->gimbal  roll={math.degrees(roll):.2f}°  "
                f"pitch={math.degrees(pitch):.2f}°  "
                f"yaw={math.degrees(yaw):.2f}°"
            )
        except Exception as e:
            self.get_logger().warn(f"Could not lookup map->gimbal transform: {e}")
        

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