import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class MassSpringVisual(Node):

    def __init__(self):
        super().__init__("mass_spring_visual")

        # Subscriber
        self.sub = self.create_subscription(
            Float64,
            "mass_position",
            self.position_callback,
            10
        )

        # Marker publisher
        self.marker_pub = self.create_publisher(
            Marker,
            "visual_marker",
            10
        )

        self.get_logger().info("Mass-Spring Visual Node Started")

    def position_callback(self, msg):
        x = msg.data
        now = self.get_clock().now().to_msg()

        # -------- Fixed block --------
        fixed = Marker()
        fixed.header.frame_id = "world"
        fixed.header.stamp = now
        fixed.ns = "fixed"
        fixed.id = 0
        fixed.type = Marker.CUBE
        fixed.action = Marker.ADD
        fixed.pose.position.x = 0.0
        fixed.pose.position.y = 0.0
        fixed.pose.position.z = 0.0
        fixed.scale.x = 0.1
        fixed.scale.y = 0.1
        fixed.scale.z = 0.1
        fixed.color.r = 1.0
        fixed.color.a = 1.0

        # -------- Mass --------
        mass = Marker()
        mass.header.frame_id = "world"
        mass.header.stamp = now
        mass.ns = "mass"
        mass.id = 1
        mass.type = Marker.SPHERE
        mass.action = Marker.ADD
        mass.pose.position.x = x
        mass.pose.position.y = 0.0
        mass.pose.position.z = 0.0
        mass.scale.x = 0.15
        mass.scale.y = 0.15
        mass.scale.z = 0.15
        mass.color.g = 1.0
        mass.color.a = 1.0

        # -------- Spring --------
        spring = Marker()
        spring.header.frame_id = "world"
        spring.header.stamp = now
        spring.ns = "spring"
        spring.id = 2
        spring.type = Marker.LINE_STRIP
        spring.action = Marker.ADD
        spring.scale.x = 0.05
        spring.color.b = 1.0
        spring.color.a = 1.0

        p1 = Point()
        p1.x = 0.0
        p1.y = 0.0
        p1.z = 0.0

        p2 = Point()
        p2.x = x
        p2.y = 0.0
        p2.z = 0.0

        spring.points.clear()
        spring.points.append(p1)
        spring.points.append(p2)

        # Publish markers
        self.marker_pub.publish(fixed)
        self.marker_pub.publish(mass)
        self.marker_pub.publish(spring)


def main(args=None):
    rclpy.init(args=args)
    node = MassSpringVisual()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

