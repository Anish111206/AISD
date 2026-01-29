#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class EnergyNode(Node):

    def __init__(self):
        super().__init__('energy_node')

        # Physical parameters (must match physics node)
        self.m = 1.0
        self.k = 5.0

        # State variables
        self.x = 5.0
        self.v = 0.0

        # Subscribers
        self.create_subscription(
            Float64,
            'mass_position',
            self.position_callback,
            10
        )

        self.create_subscription(
            Float64,
            'mass_velocity',
            self.velocity_callback,
            10
        )

        # Publishers
        self.ke_pub = self.create_publisher(Float64, '/energy/kinetic', 10)
        self.pe_pub = self.create_publisher(Float64, '/energy/potential', 10)
        self.te_pub = self.create_publisher(Float64, '/energy/total', 10)

        # Timer for computation
        self.timer = self.create_timer(0.01, self.compute_energy)

        self.get_logger().info("Energy computation node started")

    def position_callback(self, msg):
        self.x = msg.data

    def velocity_callback(self, msg):
        self.v = msg.data

    def compute_energy(self):
        # Kinetic Energy
        ke = 0.5 * self.m * self.v * self.v

        # Potential Energy
        pe = 0.5 * self.k * self.x * self.x

        # Total Energy
        te = ke + pe

        # Publish
        self.ke_pub.publish(Float64(data=ke))
        self.pe_pub.publish(Float64(data=pe))
        self.te_pub.publish(Float64(data=te))


def main(args=None):
    rclpy.init(args=args)
    node = EnergyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

