#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import SetBool


class PerturbationService(Node):

    def __init__(self):
        super().__init__('perturbation_service')

        # Publisher to external force topic
        self.force_pub = self.create_publisher(
            Float64,
            '/external_force',
            10
        )

        # Service
        self.srv = self.create_service(
            SetBool,
            '/apply_perturbation',
            self.handle_service
        )

        self.force_value = 10.0        # Newtons
        self.duration = 0.5           # seconds

        self.get_logger().info("Perturbation service ready")

    def handle_service(self, request, response):
        if request.data:
            self.get_logger().info("Applying perturbation")

            # Apply force
            msg = Float64()
            msg.data = self.force_value
            self.force_pub.publish(msg)

            # Remove force after duration
            self.create_timer(self.duration, self.reset_force)

            response.success = True
            response.message = "Perturbation applied"

        else:
            self.reset_force()
            response.success = True
            response.message = "Perturbation stopped"

        return response

    def reset_force(self):
        msg = Float64()
        msg.data = 0.0
        self.force_pub.publish(msg)
        self.get_logger().info("Perturbation removed")


def main(args=None):
    rclpy.init(args=args)
    node = PerturbationService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

