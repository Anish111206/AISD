import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class MassSpringVisual(Node):

    def __init__(self):
        super().__init__("mass_spring_visual")
        self.get_logger().info("Visual node started")

        # Subscriber: listens to mass position
        self.sub = self.create_subscription(
            Float64,
            "mass_position",
            self.position_callback,
            10
        )

        # Publisher: sends markers to RViz
        self.marker_pub = self.create_publisher(
            Marker,
            "visual_marker",
            10
        )

    def position_callback(self, msg):
        x = msg.data
        now = self.get_clock().now().to_msg()

        # Fixed point
        fixed = Marker()
        fixed.header.frame_id = "map"
        fixed.header.stamp = now
        fixed.ns = "fixed"
        fixed.id = 0
        fixed.type = Marker.CUBE
        fixed.action = Marker.ADD
        fixed.pose.position.x = 0.0
        fixed.scale.x = fixed.scale.y = fixed.scale.z = 0.1
        fixed.color.r = 1.0
        fixed.color.a = 1.0

        # Mass
        mass = Marker()
        mass.header.frame_id = "map"
        mass.header.stamp = now
        mass.ns = "mass"
        mass.id = 1
        mass.type = Marker.SPHERE
        mass.action = Marker.ADD
        mass.pose.position.x = x
        mass.scale.x = mass.scale.y = mass.scale.z = 0.1
        mass.color.g = 1.0
        mass.color.a = 1.0

        # Spring
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
        p2.x = x
        spring.points = [p1, p2]

        self.marker_pub.publish(fixed)
        self.marker_pub.publish(mass)
        self.marker_pub.publish(spring)

# ======== ADD THIS MAIN FUNCTION =========
def main(args=None):
    rclpy.init(args=args)
    node = MassSpringVisual()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

