import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class MassSpringNode(Node):

    def __init__(self):
        super().__init__("physics_node")
        self.get_logger().info("Physics node started and publishing position")

        # Publisher
        self.position_pub = self.create_publisher(
            Float64,
            "mass_position",
            10
        )

        # Physical parameters
        self.k = 10.0
        self.b = 1.0
        self.m = 1.0

        # State variables
        self.x = 0.5
        self.v = 0.0

        # Time step
        self.dt = 0.01

        # Timer
        self.timer = self.create_timer(self.dt, self.update)



    def update(self):
        # Physics
        spring_force = -self.k * self.x
        damping_force = -self.b * self.v
        force = spring_force + damping_force
        a = force / self.m

        self.v += a * self.dt
        self.x += self.v * self.dt

        # Publish
        msg = Float64()
        msg.data = self.x
        self.position_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MassSpringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

