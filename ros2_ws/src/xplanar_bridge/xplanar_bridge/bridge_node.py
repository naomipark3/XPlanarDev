import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from xplanar_interfaces.action import MoveTo, TiltTo, RotateTo, MoveZ
from .mover_control import XPlanarController
from geometry_msgs.msg import PoseStamped, Quaternion
import math


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

        self._init_service = self.create_service(
            Trigger, 'initialize', self.handle_initialize
        )

        self._move_server = ActionServer(
            self,
            MoveTo,
            'move_to',
            execute_callback=self.execute_move,
        )

        self._tilt_server = ActionServer(
            self, TiltTo, 'tilt_to', execute_callback=self.execute_tilt,
        )
        self._rotate_server = ActionServer(
            self, RotateTo, 'rotate_to', execute_callback=self.execute_rotate,
        )
        self._z_server = ActionServer(
            self, MoveZ, 'move_z', execute_callback=self.execute_move_z,
        )
        
        self.get_logger().info("Service '/initialize' ready")
        self.get_logger().info("Action server 'move_to' ready")
        self.get_logger().info("Action servers 'tilt_to' and 'rotate_to' ready")
        self.get_logger().info("Action server 'move_z' ready")

    def handle_initialize(self, request, response):
        self.get_logger().info("Initialize requested (will take ~13s)")
        try:
            self.ctrl.initialize()
            response.success = True
            response.message = "System initialized, movers lifted"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Initialize failed: {e}"
            self.get_logger().error(response.message)
        return response

    def publish_states(self):
        for mid, pose in self.ctrl.get_all_mover_positions().items():
            x, y, z, a, b, c = pose
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'xplanar'
            msg.pose.position.x = x / 1000.0
            msg.pose.position.y = y / 1000.0
            msg.pose.position.z = z / 1000.0
            msg.pose.orientation = self._rpy_to_quat(a, b, c)
            self.pubs[mid].publish(msg)

    @staticmethod
    def _rpy_to_quat(roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

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

    def execute_tilt(self, goal_handle):
        g = goal_handle.request
        self.get_logger().info(
            f"Tilt request: mover {g.mover_id} axis {g.axis} -> {g.angle:.4f} rad"
        )
        velocity = g.velocity if g.velocity > 0 else 0.5

        def on_progress(current_angle):
            fb = TiltTo.Feedback()
            fb.current_angle = current_angle
            goal_handle.publish_feedback(fb)

        result_data = self.ctrl.tilt_to(
            g.mover_id, g.angle, axis=g.axis,
            velocity=velocity, on_progress=on_progress,
        )

        result = TiltTo.Result()
        result.success = result_data.success
        result.error_id = result_data.error_id
        result.message = result_data.message

        # read final angle from the appropriate axis
        pose = self.ctrl.get_all_mover_positions()[g.mover_id]
        result.final_angle = pose[3] if g.axis.upper() == "A" else pose[4]

        if result_data.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def execute_rotate(self, goal_handle):
        g = goal_handle.request
        self.get_logger().info(
            f"Rotate request: mover {g.mover_id} -> {g.angle:.4f} rad "
            f"(+{g.additional_turns} turns)"
        )
        velocity = g.velocity if g.velocity > 0 else 1.0

        def on_progress(current_angle):
            fb = RotateTo.Feedback()
            fb.current_angle = current_angle
            goal_handle.publish_feedback(fb)

        result_data = self.ctrl.rotate_to(
            g.mover_id, g.angle, additional_turns=g.additional_turns,
            velocity=velocity, on_progress=on_progress,
        )

        result = RotateTo.Result()
        result.success = result_data.success
        result.error_id = result_data.error_id
        result.message = result_data.message
        result.final_angle = self.ctrl.get_all_mover_positions()[g.mover_id][5]

        if result_data.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def execute_move_z(self, goal_handle):
        g = goal_handle.request
        self.get_logger().info(
            f"Z move request: mover {g.mover_id} -> {g.z:.4f} mm"
        )
        velocity = g.velocity if g.velocity > 0 else 50.0

        def on_progress(current_z):
            fb = MoveZ.Feedback()
            fb.current_z = current_z
            goal_handle.publish_feedback(fb)

        result_data = self.ctrl.move_z(
            g.mover_id, g.z,
            velocity=velocity, on_progress=on_progress,
        )

        result = MoveZ.Result()
        result.success = result_data.success
        result.error_id = result_data.error_id
        result.message = result_data.message
        result.final_z = self.ctrl.get_all_mover_positions()[g.mover_id][2]

        if result_data.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


def main():
    rclpy.init()
    node = XPlanarBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
