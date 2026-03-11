import argparse
import os
import sys
import tty
import termios
import select
import yaml
from datetime import datetime
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def getch(timeout_sec: float = 0.1) -> Optional[str]:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout_sec)
        if rlist:
            ch = os.read(fd, 3).decode(errors='ignore')
            return ch
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def prompt_confirm(prompt: str) -> bool:
    sys.stdout.write(f"{prompt} [y/N]: ")
    sys.stdout.flush()
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        ch = sys.stdin.read(1)
        sys.stdout.write("\n")
        return ch.lower() == 'y'
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


class JointStatesBuffer:
    def __init__(self) -> None:
        self.joint_names: List[str] = []
        self.points: List[List[float]] = []
        self.last_msg: Optional[JointState] = None

    def update(self, msg: JointState) -> None:
        self.last_msg = msg
        if not self.joint_names:
            self.joint_names = list(msg.name)

    def snapshot(self) -> Optional[List[float]]:
        if self.last_msg is None:
            return None
        if not self.joint_names:
            self.joint_names = list(self.last_msg.name)
        name_to_pos = dict(zip(self.last_msg.name, self.last_msg.position))
        return [float(name_to_pos.get(n, 0.0)) for n in self.joint_names]


class JointStatesRecorder(Node):
    def __init__(self, joint_states_topic: str, rate_hz: float) -> None:
        super().__init__('record_joint_states_always')
        self.buffer = JointStatesBuffer()
        self.subscription = self.create_subscription(
            JointState, joint_states_topic, self._on_joint_state, 10
        )
        self.is_recording = False
        self.timer_period = 1.0 / max(rate_hz, 1e-3)
        self.timer = self.create_timer(self.timer_period, self._on_timer)

    def _on_joint_state(self, msg: JointState) -> None:
        self.buffer.update(msg)

    def _on_timer(self) -> None:
        if not self.is_recording:
            return
        snap = self.buffer.snapshot()
        if snap is not None:
            self.buffer.points.append(snap)


def save_yaml(file_path: str, joint_names: List[str], points: List[List[float]], dt: float) -> None:
    data = {
        'joint_names': joint_names,
        'points': [
            {
                'positions': p,
                'time_from_start': float((i + 1) * dt),
            }
            for i, p in enumerate(points)
        ],
    }
    with open(file_path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)


def default_filename(prefix: str = 'joint_states_stream') -> str:
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    return f"{prefix}_{stamp}.yaml"


def main() -> None:
    parser = argparse.ArgumentParser(description='Continuously record /joint_states to YAML')
    parser.add_argument('--topic', default='/joint_states', help='Joint states topic name')
    parser.add_argument('--outfile', default='', help='Output YAML file path')
    parser.add_argument('--rate', type=float, default=10.0, help='Recording rate in Hz when running')
    args = parser.parse_args()

    rclpy.init()
    node = JointStatesRecorder(args.topic, args.rate)

    dt = 1.0 / max(args.rate, 1e-3)

    print('Controls:')
    print('- SPACE: start/stop recording toggle')
    print('- p: pause/resume recording (same as SPACE)')
    print('- c: clear all (with confirm)')
    print('- w: save and quit (with confirm)')
    print('- q: quit without saving')

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            key = getch(0.05)
            if not key:
                continue
            if key == ' ' or key.lower() == 'p':
                node.is_recording = not node.is_recording
                print('Recording' if node.is_recording else 'Paused')
            elif key.lower() == 'c':
                if not node.buffer.points:
                    print('Buffer already empty.')
                else:
                    if prompt_confirm('Clear ALL records?'):
                        node.buffer.points.clear()
                        print('Cleared all records.')
            elif key.lower() == 'w':
                if not node.buffer.points:
                    print('Nothing to save.')
                    continue
                if prompt_confirm('Save and quit?'):
                    out = args.outfile or default_filename()
                    save_yaml(out, node.buffer.joint_names, node.buffer.points, dt=dt)
                    print(f'Saved to {out}')
                    break
            elif key.lower() == 'q':
                print('Quit without saving.')
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 