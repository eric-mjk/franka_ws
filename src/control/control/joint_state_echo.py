import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateEcho(Node):
    def __init__(self):
        super().__init__("joint_state_echo")
        self.create_subscription(JointState, "/franka/joint_states", self.cb, 10)
        self.get_logger().info("Listening to /franka/joint_states ...")

    def cb(self, msg: JointState):
        # Print only first 7 joints (arm), nicely
        pairs = list(zip(msg.name, msg.position))
        arm = [p for p in pairs if p[0].startswith("fr3_joint")]
        arm = sorted(arm, key=lambda x: x[0])  # joint1..joint7

        if not arm:
            self.get_logger().info(f"Got JointState with names: {msg.name[:5]} ... (no fr3_joint*)")
            return

        line = "\n"+"\n".join([f"{name}:{pos:+.3f}" for name, pos in arm])
        self.get_logger().info(line)


def main():
    rclpy.init()
    node = JointStateEcho()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()