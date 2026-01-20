import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class MassSpringPhysics(Node):

    def __init__(self):
        super().__init__("mass_spring_physics")

        # Publisher: sends mass position
        self.position_pub = self.create_publisher(
            Float64,
            "mass_position",
            10
        )

        # Physical parameters
        self.k = 10.0   # spring constant
        self.b = 1.0    # damping constant
        self.m = 1.0    # mass

        # State variables
        self.x = 0.5    # initial displacement
        self.v = 0.0    # initial velocity

        # Simulation timestep
        self.dt = 0.01  # seconds

        # Timer → calls update() every dt seconds
        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info("Mass-Spring Physics Node Started")

    def update(self):
        # Forces
        spring_force = -self.k * self.x
        damping_force = -self.b * self.v
        force = spring_force + damping_force

        # Newton's second law
        a = force / self.m

        # Euler integration
        self.v += a * self.dt
        self.x += self.v * self.dt

        # Publish position
        msg = Float64()
        msg.data = self.x
        self.position_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MassSpringPhysics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

