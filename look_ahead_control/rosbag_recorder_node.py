"""Rosbag auto-recorder node triggered by MAVLink ARM+AUTO mode."""

from datetime import datetime
import os
import signal
import subprocess

from bme_common_msgs.msg import MavModes
import rclpy
from rclpy.node import Node

ARDUPILOT_AUTO_BASE = 217
ARDUPILOT_AUTO_CUSTOM = 10


class RosbagRecorder(Node):
    """Record rosbag automatically when vehicle enters ARM+AUTO mode."""

    def __init__(self):
        """Initialize rosbag recorder node."""
        super().__init__('rosbag_recorder')

        self.declare_parameter('bag_output_dir', '~/rosbag2')
        self.declare_parameter('exclude_topics', '')

        self._recording = False
        self._bag_proc = None

        self.create_subscription(
            MavModes, '/mav/modes', self._modes_cb, 10)

        self.get_logger().info('rosbag_recorder ready')

    def _modes_cb(self, msg):
        armed_auto = (
            msg.base_mode == ARDUPILOT_AUTO_BASE
            and msg.custom_mode == ARDUPILOT_AUTO_CUSTOM
        )

        if armed_auto and not self._recording:
            self._start_recording()
        elif not armed_auto and self._recording:
            self._stop_recording()

    def _start_recording(self):
        output_dir = self.get_parameter(
            'bag_output_dir').get_parameter_value().string_value
        output_dir = os.path.expanduser(output_dir)
        os.makedirs(output_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_name = os.path.join(output_dir, f'auto_{timestamp}')

        exclude = self.get_parameter(
            'exclude_topics').get_parameter_value().string_value

        cmd = ['ros2', 'bag', 'record', '-a', '-o', bag_name]
        if exclude:
            cmd.extend(['--exclude', exclude])

        self.get_logger().info(f'Starting bag recording: {bag_name}')
        self._bag_proc = subprocess.Popen(
            cmd,
            preexec_fn=os.setsid,
        )
        self._recording = True

    def _stop_recording(self):
        if self._bag_proc is not None:
            self.get_logger().info('Stopping bag recording')
            try:
                os.killpg(os.getpgid(self._bag_proc.pid), signal.SIGINT)
                self._bag_proc.wait(timeout=10)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                self.get_logger().warn('Bag process did not exit, killing')
                os.killpg(os.getpgid(self._bag_proc.pid), signal.SIGKILL)
                self._bag_proc.wait()
            self._bag_proc = None
        self._recording = False

    def destroy_node(self):
        """Stop recording before shutdown."""
        if self._recording:
            self._stop_recording()
        super().destroy_node()


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = RosbagRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
