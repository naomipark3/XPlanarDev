import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped
from xplanar_interfaces.action import MoveTo
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
        self.get_logger().info("Connected to PLC")

        self.pubs = {
            mid: self.create_publisher(PoseStamped, f'mover_{mid}/pose', 10)
            for mid in range(1, self.ctrl.NUM_MOVERS + 1)
        }
        self.timer = self.create_timer(0.1, self.publish_states)  #10 Hz

        self._move_server = ActionServer(
            self,
            MoveTo,
            'move_to',
            execute_callback=self.execute_move,
        )
        self.get_logger().info("Action server 'move_to' ready")

    def publish_states(self):
        for mid, (x, y) in self.ctrl.get_all_mover_positions().items():
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'xplanar'
            msg.pose.position.x = x / 1000.0  #**convert mm -> m, as this is standard ROS convention
            msg.pose.position.y = y / 1000.0
            self.pubs[mid].publish(msg)

    def execute_move(self, goal_handle):
        g = goal_handle.request
        self.get_logger().info(
            f"Move request: mover {g.mover_id} -> ({g.x:.1f}, {g.y:.1f})"
        )

        velocity = g.velocity if g.velocity > 0 else 300.0
        goal = (g.x, g.y)

        def on_progress(status):
            fb = MoveTo.Feedback()
            fb.current_x = status.x
            fb.current_y = status.y
            dx, dy = goal[0] - status.x, goal[1] - status.y
            fb.distance_remaining = (dx * dx + dy * dy) ** 0.5
            goal_handle.publish_feedback(fb)

        result_data = self.ctrl.smart_move_to(
            g.mover_id, g.x, g.y, velocity=velocity, on_progress=on_progress
        )

        result = MoveTo.Result()
        result.success = result_data.success
        result.error_id = result_data.error_id
        result.message = result_data.message
        result.final_x = result_data.final_x
        result.final_y = result_data.final_y

        if result_data.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def destroy_node(self):
        ''''This should be called upon ctrl+c for safe exiting'''
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
