from vdes_interfaces.msg import PointList

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point

from random import random

class TargetGeneration(Node):
    def __init__(self):
        super().__init__('target_generation')

        self.set_up_parameters()
        self.set_up_service()
        self.generate_target()
        self.set_up_timer()

        self.get_logger().info("Géneration des cibles démarrée.")

    def set_up_parameters(self):
        # Number of target to generate
        self.declare_parameter('nmb_target', 10)
        self.nmb_target = self.get_parameter('nmb_target').value

        # Minimum relative north position
        self.declare_parameter('pos_n_min', 50.0)
        self.pos_n_min = self.get_parameter('pos_n_min').value

        # Maximum relative north position
        self.declare_parameter('pos_n_max', 100.0)
        self.pos_n_max = self.get_parameter('pos_n_max').value

        # Minimum relative east position
        self.declare_parameter('pos_e_min', 50.0)
        self.pos_e_min = self.get_parameter('pos_e_min').value

        # Maximum relative east position
        self.declare_parameter('pos_e_max', 100.0)
        self.pos_e_max = self.get_parameter('pos_e_max').value


    def set_up_service(self):
        self.targets_pub = self.create_publisher(PointList, 'get_targets', 10)

    def generate_target(self):
        self.targets = []
        for i in range(self.nmb_target):
            self.targets.append(Point(x=self.pos_e_min+(self.pos_e_max-self.pos_e_min)*random(),
                                      y=self.pos_n_min+(self.pos_n_max-self.pos_n_min)*random()))

    def set_up_timer(self):
        self.timer = self.create_timer(0.1, self.get_targets_callback)

    def get_targets_callback(self):
        msg = PointList()
        msg.points = self.targets
        self.targets_pub.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    target_generation_server = TargetGeneration()
    rclpy.spin(target_generation_server)
    rclpy.shutdown()

if __name__=="__main__":
    main()

    

