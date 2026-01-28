import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class MassSpringNode(Node):

    def __init__(self):
        super().__init__("mass_spring_node")
        self.get_logger().info("Mass-Spring node started |")

        # Only ONE publisher (RViz)
        self.marker_pub = self.create_publisher(
            Marker,
            "visual_marker",
            10
        )

        # Physics parameters
        self.k = 5.0
        self.b = 0.5
        self.m = 1.0

        self.x = 5.0
        self.v = 0.0
        self.dt = 0.01

        # Timer
        self.timer = self.create_timer(self.dt, self.update)

    def update(self):
        # ---------- Physics ----------
        spring_force = -self.k * self.x
        damping_force = -self.b * self.v
        force = spring_force + damping_force
        a = force / self.m

        self.v += a * self.dt
        self.x += self.v * self.dt

        now = self.get_clock().now().to_msg()

        # ---------- Fixed point ----------
        fixed = Marker()
        fixed.header.frame_id = "map"
        fixed.header.stamp = now
        fixed.ns = "fixed"
        fixed.id = 0
        fixed.type = Marker.CUBE
        fixed.action = Marker.ADD
        fixed.scale.x = fixed.scale.y = fixed.scale.z = 0.1
        fixed.color.r = 1.0
        fixed.color.a = 1.0

        # ---------- Mass ----------
        mass = Marker()
        mass.header.frame_id = "map"
        mass.header.stamp = now
        mass.ns = "mass"
        mass.id = 1
        mass.type = Marker.SPHERE
        mass.action = Marker.ADD
        mass.pose.position.x = self.x
        mass.scale.x = mass.scale.y = mass.scale.z = 0.1
        mass.color.g = 1.0
        mass.color.a = 1.0

        # ---------- Spring ----------
        spring = Marker()
        spring.header.frame_id = "map"
        spring.header.stamp = now
        spring.ns = "spring"
        spring.id = 2
        spring.type = Marker.LINE_STRIP
        spring.action = Marker.ADD
        spring.scale.x = 0.02
        spring.color.b = 1.0
        spring.color.a = 1.0

        p1 = Point()
        p1.x = 0.0
        p2 = Point()
        p2.x = self.x
        spring.points = [p1, p2]

        # Publish
        self.marker_pub.publish(fixed)
        self.marker_pub.publish(mass)
        self.marker_pub.publish(spring)


def main(args=None):
    rclpy.init(args=args)
    node = MassSpringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
