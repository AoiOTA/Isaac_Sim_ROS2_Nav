"""Interactive W/A/S/D terminal adapter for the safe teleop policy."""

from __future__ import annotations

import os
import select
import signal
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy

from robot_teleop.safety import MotionCommand
from robot_teleop.safety import TeleopConfig
from robot_teleop.safety import TeleopController
from robot_teleop.safety import TeleopRuntime


HELP = """
Isaac Nav - Mapping Teleop
  W / Up       forward          S / Down     reverse
  A / Left     turn left        D / Right    turn right
  Space        stop now         Q / Ctrl+C/D stop and exit

Hold or repeat a motion key. The wall-time deadman stops the robot within
0.20 seconds if key events stop. This node is only for Mapping and
Incremental Mapping; never run it alongside Navigation.
""".strip()


def decode_keypresses(data: bytes) -> list[str]:
    """Decode ASCII and terminal arrow sequences into policy key names."""
    keys: list[str] = []
    arrows = {
        b'\x1b[A': 'up',
        b'\x1b[B': 'down',
        b'\x1b[C': 'right',
        b'\x1b[D': 'left',
    }
    index = 0
    while index < len(data):
        sequence = data[index:index + 3]
        if sequence in arrows:
            keys.append(arrows[sequence])
            index += 3
            continue
        value = data[index:index + 1]
        try:
            keys.append(value.decode('ascii'))
        except UnicodeDecodeError:
            keys.append('unknown')
        index += 1
    return keys


class RawTerminal:
    """Restore terminal attributes even when teleop exits exceptionally."""

    def __init__(self, stream) -> None:
        """Remember the interactive stream without changing it yet."""
        self._stream = stream
        self._fd = stream.fileno()
        self._settings = None

    def __enter__(self):
        """Switch the terminal to raw input mode."""
        if not self._stream.isatty():
            raise RuntimeError('keyboard teleop requires an interactive TTY')
        self._settings = termios.tcgetattr(self._fd)
        tty.setraw(self._fd)
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        """Restore the original terminal settings."""
        del exc_type, exc_value, traceback
        if self._settings is not None:
            termios.tcsetattr(
                self._fd, termios.TCSADRAIN, self._settings)

    def read(self, timeout_sec: float) -> list[str]:
        """Read all currently available bytes within a bounded wall timeout."""
        readable, _, _ = select.select(
            [self._fd], [], [], max(0.0, timeout_sec))
        if not readable:
            return []
        data = os.read(self._fd, 32)
        return decode_keypresses(data) if data else ['eof']


class KeyboardTeleopNode(Node):
    """Publish bounded mapping commands without depending on ROS time."""

    def __init__(self) -> None:
        """Validate parameters and create the sole `/cmd_vel` publisher."""
        super().__init__('keyboard_teleop')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('command_timeout_sec', 0.18)
        self.declare_parameter('linear_speed', 0.30)
        self.declare_parameter('angular_speed', 0.60)
        self.declare_parameter('max_linear_speed', 1.00)
        self.declare_parameter('max_angular_speed', 1.50)

        self.publish_rate_hz = float(
            self.get_parameter('publish_rate_hz').value)
        if not 10.0 <= self.publish_rate_hz <= 100.0:
            raise ValueError('publish_rate_hz must be within [10, 100]')
        topic = str(self.get_parameter('cmd_vel_topic').value).strip()
        if topic != '/cmd_vel':
            raise ValueError(
                'mapping teleop cmd_vel_topic must remain exactly /cmd_vel')

        config = TeleopConfig(
            linear_speed=float(self.get_parameter('linear_speed').value),
            angular_speed=float(self.get_parameter('angular_speed').value),
            max_linear_speed=float(
                self.get_parameter('max_linear_speed').value),
            max_angular_speed=float(
                self.get_parameter('max_angular_speed').value),
            command_timeout_sec=float(
                self.get_parameter('command_timeout_sec').value),
        )
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(Twist, topic, qos)
        self.runtime = TeleopRuntime(
            TeleopController(config), self._publish_motion)

    def _publish_motion(self, command: MotionCommand) -> None:
        message = Twist()
        message.linear.x = command.linear_x
        message.angular.z = command.angular_z
        self._publisher.publish(message)


def _raise_keyboard_interrupt(signum, frame) -> None:
    del signum, frame
    raise KeyboardInterrupt


def _run(node: KeyboardTeleopNode) -> None:
    period = 1.0 / node.publish_rate_hz
    next_publish = time.monotonic()
    last_displayed: MotionCommand | None = None
    print(HELP, flush=True)

    with RawTerminal(sys.stdin) as terminal:
        while rclpy.ok():
            now = time.monotonic()
            wait = min(0.05, max(0.0, next_publish - now))
            should_exit = False
            for key in terminal.read(wait):
                if node.runtime.handle_key(key, time.monotonic()):
                    should_exit = True
                    break
            if should_exit:
                return

            now = time.monotonic()
            if now < next_publish:
                continue
            command = node.runtime.tick(now)
            next_publish = now + period
            if command != last_displayed:
                print(
                    '\rlinear.x={:+.2f} m/s  angular.z={:+.2f} rad/s   '
                    .format(command.linear_x, command.angular_z),
                    end='',
                    flush=True,
                )
                last_displayed = command


def main(args=None) -> None:
    """Run mapping teleop and guarantee a final zero command."""
    rclpy.init(args=args)
    node: KeyboardTeleopNode | None = None
    previous_handlers = {}
    try:
        node = KeyboardTeleopNode()
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous_handlers[signum] = signal.signal(
                signum, _raise_keyboard_interrupt)
        _run(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.runtime.close()
            print(
                '\nTeleop stopped; final zero velocity published.',
                flush=True,
            )
            node.destroy_node()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
