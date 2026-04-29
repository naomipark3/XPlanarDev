import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from .mover_control import XPlanarController


class XPlanarBridge(Node):
    def __init__(self):
        super().__init__('xplanar_bridge')
        self.ctrl = XPlanarController(
            ams_net_id="169.254.137.138.1.1",
            port=852,
            local_ip="192.168.1.1",
        )
        self.ctrl.connect()

        self.pubs = {
            mid: self.create_publisher(PoseStamped, f'mover_{mid}/pose', 10)
            for mid in range(1, self.ctrl.NUM_MOVERS + 1)
        }
        self.timer = self.create_timer(0.1, self.publish_states)  # 10 Hz

    def publish_states(self):
        for mid, (x, y) in self.ctrl.get_all_mover_positions().items():
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'xplanar'
            msg.pose.position.x = x / 1000.0  #**convert mm -> m, as this is standard ROS convention
            msg.pose.position.y = y / 1000.0
            self.pubs[mid].publish(msg)

    def destroy_node(self):
        ''''This should be called upon ctrl+c for save exiting'''
        self.ctrl.disconnect()
        super().destroy_node()


def main():
    rclpy.init()
    node = XPlanarBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
