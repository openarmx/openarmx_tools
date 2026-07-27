import argparse
import sys
import time
import yaml
import asyncio
import threading
from typing import List, Dict

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray


async def _wait_for_rclpy_future(rclpy_future):
    """
    Safely wait for a rclpy Future by creating an asyncio Future bridge.
    This ensures compatibility with asyncio.gather().
    """
    loop = asyncio.get_event_loop()
    asyncio_future = loop.create_future()

    def done_callback(future):
        try:
            result = future.result()
            loop.call_soon_threadsafe(asyncio_future.set_result, result)
        except Exception as e:
            loop.call_soon_threadsafe(asyncio_future.set_exception, e)

    rclpy_future.add_done_callback(done_callback)
    return await asyncio_future


def load_yaml(file_path: str) -> Dict:
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def filter_joints(joint_names: List[str], points: List[Dict], target_joints: List[str]) -> tuple:
    """Filter trajectory to only include target joints in the correct order."""
    if not target_joints:
        return joint_names, points
    
    # Find indices of target joints in the correct order
    joint_indices = []
    filtered_joint_names = []
    for target_joint in target_joints:
        if target_joint in joint_names:
            idx = joint_names.index(target_joint)
            joint_indices.append(idx)
            filtered_joint_names.append(target_joint)
        else:
            print(f"Warning: Joint '{target_joint}' not found in recorded joints")
    
    if not joint_indices:
        print("Error: No target joints found in recorded trajectory")
        return [], []
    
    # Filter points
    filtered_points = []
    for point in points:
        positions = point.get('positions', [])
        if len(positions) >= len(joint_names):
            filtered_positions = [positions[i] for i in joint_indices]
            filtered_point = {
                'positions': filtered_positions,
                'time_from_start': point.get('time_from_start', 0.0)
            }
            filtered_points.append(filtered_point)
    
    return filtered_joint_names, filtered_points


class _TrajectoryActionClient:
    def __init__(self, action_name: str, node: Node) -> None:
        self.action_client = ActionClient(node, FollowJointTrajectory, action_name)
        self.action_name = action_name
        self.node = node
        self._latest_feedback_time: float = 0.0  # seconds since trajectory start (desired time_from_start)

    def _on_feedback(self, feedback_msg: FollowJointTrajectory.Feedback) -> None:
        """Feedback callback from ActionClient.

        In rclpy the callback actually receives a *FeedbackMessage* wrapper whose
        structure is: <ActionName>_FeedbackMessage(goal_id, feedback=<Feedback>). For
        FollowJointTrajectory the real feedback is under .feedback and contains the
        fields: desired, actual, error (each a JointTrajectoryPoint). Older code
        assumed direct access (feedback_msg.desired) which raises AttributeError.
        We unwrap defensively to support both forms.
        """
        try:
            # Newer rclpy: wrapper with .feedback
            fb = getattr(feedback_msg, 'feedback', feedback_msg)
            desired = getattr(fb, 'desired', None)
            if desired is None:
                return
            t = desired.time_from_start
            self._latest_feedback_time = float(t.sec) + float(t.nanosec) / 1e9
        except Exception:
            # Silently ignore malformed feedback
            pass

    def get_latest_feedback_time(self) -> float:
        return self._latest_feedback_time

    async def send_trajectory_async(self, joint_names: List[str], points: List[Dict], rate_scale: float = 1.0) -> bool:
        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(f'Action server {self.action_name} not available')
            return False

        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        for pt in points:
            jp = JointTrajectoryPoint()
            jp.positions = [float(x) for x in pt.get('positions', [])]
            t = float(pt.get('time_from_start', 0.0)) / max(rate_scale, 1e-6)
            jp.time_from_start = rclpy.duration.Duration(seconds=t).to_msg()
            trajectory.points.append(jp)

        # Create action goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.node.get_logger().info(f'Sending trajectory with {len(trajectory.points)} points to {self.action_name}')

        # Send goal and wait for result
        # Register feedback callback to track execution progress for synchronization.
        # Use bridge function to ensure rclpy Future compatibility with asyncio
        future = self.action_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        goal_handle = await _wait_for_rclpy_future(future)

        if goal_handle.accepted:
            self.node.get_logger().info(f'Goal accepted by {self.action_name}!')
            result_future = goal_handle.get_result_async()
            result = await _wait_for_rclpy_future(result_future)
            self.node.get_logger().info(f'{self.action_name} result: {result.result.error_string}')
            return result.result.error_code == 0
        else:
            self.node.get_logger().error(f'Goal rejected by {self.action_name}')
            return False


class _TrajectoryTopicPublisher:
    """Trajectory control using topic instead of action (for multi-robot setups)."""
    def __init__(self, topic_name: str, node: Node) -> None:
        self.publisher = node.create_publisher(JointTrajectory, topic_name, 10)
        self.topic_name = topic_name
        self.node = node
        self._latest_feedback_time: float = 0.0

    def get_latest_feedback_time(self) -> float:
        return self._latest_feedback_time

    async def send_trajectory_async(self, joint_names: List[str], points: List[Dict], rate_scale: float = 1.0) -> bool:
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        for pt in points:
            jp = JointTrajectoryPoint()
            jp.positions = [float(x) for x in pt.get('positions', [])]
            t = float(pt.get('time_from_start', 0.0)) / max(rate_scale, 1e-6)
            jp.time_from_start = rclpy.duration.Duration(seconds=t).to_msg()
            trajectory.points.append(jp)

        self.node.get_logger().info(f'Publishing trajectory with {len(trajectory.points)} points to {self.topic_name}')

        # Publish trajectory
        self.publisher.publish(trajectory)

        # Simulate feedback for synchronization
        if trajectory.points:
            playback_start = time.monotonic()
            last_point_time = float(trajectory.points[-1].time_from_start.sec) + \
                            float(trajectory.points[-1].time_from_start.nanosec) / 1e9

            # Update feedback time progressively
            while True:
                elapsed = time.monotonic() - playback_start
                self._latest_feedback_time = min(elapsed, last_point_time)
                if elapsed >= last_point_time:
                    break
                await asyncio.sleep(0.05)

        self.node.get_logger().info(f'{self.topic_name} trajectory published successfully')
        return True


class _GripperActionClient:
    def __init__(self, action_name: str, node: Node) -> None:
        self.action_client = ActionClient(node, GripperCommand, action_name)
        self.action_name = action_name
        self.node = node

    async def send_gripper_commands_async(self, joint_names: List[str], points: List[Dict], rate_scale: float = 1.0,
                                          sync_feedback: bool = False, progress_providers: List = None,
                                          sync_margin: float = 0.0) -> bool:
        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(f'Action server {self.action_name} not available')
            return False
        
        if not joint_names or not points:
            self.node.get_logger().warning(f'No gripper data for {self.action_name}')
            return True
        # Determine a scalar gripper position from possibly multiple finger joints.
        # Strategy: use average of provided finger joint positions. This keeps symmetry
        # and works whether one or two finger joints were recorded.
        def extract_scalar(pos_list: List[float]) -> float:
            vals = []
            for idx in range(len(joint_names)):
                if idx < len(pos_list):
                    vals.append(float(pos_list[idx]))
            return sum(vals) / len(vals) if vals else 0.0

        # Compress points: keep meaningful position changes or enforce min time gap.
        compressed: List[Dict] = []
        last_pos = None
        POSITION_EPS = 1e-4  # ignore tiny noise
        MIN_TIME_GAP = 0.2   # at least 0.2s between goals to avoid action queue buildup
        last_keep_time = None
        for pt in points:
            positions = pt.get('positions', [])
            pos = extract_scalar(positions)
            t = float(pt.get('time_from_start', 0.0)) / max(rate_scale, 1e-6)
            gap_ok = (last_keep_time is None) or (t - last_keep_time >= MIN_TIME_GAP)
            change_ok = (last_pos is None) or (abs(pos - last_pos) > POSITION_EPS)
            if change_ok or gap_ok:
                compressed.append({'position': pos, 'time': t})
                last_pos = pos
                last_keep_time = t

        if not compressed:
            self.node.get_logger().warning(f'{self.action_name}: No gripper motion after compression')
            return True

        self.node.get_logger().info(f'{self.action_name}: Compressed {len(points)} raw points -> {len(compressed)} gripper goals')

        # New synchronization approach:
        # Dispatch goals at their recorded times relative to a common playback start,
        # instead of waiting for each result before scheduling the next.
        playback_start = time.monotonic()  # fallback wall clock start
        result_futures = []
        progress_providers = progress_providers or []
        FEEDBACK_POLL_DT = 0.05
        FEEDBACK_STALL_TIMEOUT = 2.0  # seconds with no usable feedback before falling back to wall clock scheduling

        last_feedback_progress = 0.0
        last_feedback_wall_time = time.monotonic()

        for i, item in enumerate(compressed):
            target_time = item['time']  # desired trajectory-relative time
            while True:
                now = time.monotonic()
                # Collect progress times from providers (may be empty early on)
                progresses = []
                if sync_feedback and progress_providers:
                    for fn in progress_providers:
                        try:
                            progresses.append(float(fn()))
                        except Exception:
                            continue
                    progresses = [p for p in progresses if p is not None]
                if sync_feedback and progresses:
                    current_progress = max(progresses)  # use max so we wait for slowest arm (safer for dual-arm)
                    # Update stall watchdog bookkeeping
                    if current_progress > last_feedback_progress + 1e-9:
                        last_feedback_progress = current_progress
                        last_feedback_wall_time = now
                    # Check if we reached scheduled time (allow margin)
                    if current_progress + sync_margin >= target_time:
                        break
                    # Not yet: sleep briefly and re-check
                    await asyncio.sleep(FEEDBACK_POLL_DT)
                    # If feedback stalled too long, fall back to wall clock vs target
                    if (now - last_feedback_wall_time) > FEEDBACK_STALL_TIMEOUT:
                        self.node.get_logger().warn(f'{self.action_name}: feedback stalled > {FEEDBACK_STALL_TIMEOUT}s, falling back to wall clock for remaining goals')
                        sync_feedback = False  # disable feedback sync for remainder
                else:
                    # Wall clock scheduling: wait until playback_start + target_time
                    elapsed = now - playback_start
                    remaining = target_time - elapsed
                    if remaining <= 0:
                        break
                    await asyncio.sleep(min(remaining, FEEDBACK_POLL_DT))

            goal = GripperCommand.Goal()
            goal.command.position = float(item['position'])
            goal.command.max_effort = 50.0
            goal_future = self.action_client.send_goal_async(goal)
            result_futures.append((i, goal.command.position, goal_future))

        # Await all results
        for i, pos, gf in result_futures:
            goal_handle = await _wait_for_rclpy_future(gf)
            if not goal_handle.accepted:
                self.node.get_logger().error(f'{self.action_name} goal {i} rejected (pos={pos})')
                continue
            result = await _wait_for_rclpy_future(goal_handle.get_result_async())
            if not result.result.reached_goal:
                self.node.get_logger().warning(f'{self.action_name} goal {i}: target not reached (pos={pos})')

        self.node.get_logger().info(f'{self.action_name}: Gripper command sequence completed (dispatched {len(compressed)} goals)')
        return True


class _GripperTopicPublisher:
    """Gripper control using topic instead of action (for multi-robot setups)."""
    def __init__(self, topic_name: str, node: Node) -> None:
        self.publisher = node.create_publisher(Float64MultiArray, topic_name, 10)
        self.topic_name = topic_name
        self.node = node

    async def send_gripper_commands_async(self, joint_names: List[str], points: List[Dict], rate_scale: float = 1.0,
                                          sync_feedback: bool = False, progress_providers: List = None,
                                          sync_margin: float = 0.0) -> bool:
        if not joint_names or not points:
            self.node.get_logger().warning(f'No gripper data for {self.topic_name}')
            return True

        # Extract scalar gripper position
        def extract_scalar(pos_list: List[float]) -> float:
            vals = []
            for idx in range(len(joint_names)):
                if idx < len(pos_list):
                    vals.append(float(pos_list[idx]))
            return sum(vals) / len(vals) if vals else 0.0

        # Compress points
        compressed: List[Dict] = []
        last_pos = None
        POSITION_EPS = 1e-4
        MIN_TIME_GAP = 0.2
        last_keep_time = None
        for pt in points:
            positions = pt.get('positions', [])
            pos = extract_scalar(positions)
            t = float(pt.get('time_from_start', 0.0)) / max(rate_scale, 1e-6)
            gap_ok = (last_keep_time is None) or (t - last_keep_time >= MIN_TIME_GAP)
            change_ok = (last_pos is None) or (abs(pos - last_pos) > POSITION_EPS)
            if change_ok or gap_ok:
                compressed.append({'position': pos, 'time': t})
                last_pos = pos
                last_keep_time = t

        if not compressed:
            self.node.get_logger().warning(f'{self.topic_name}: No gripper motion after compression')
            return True

        self.node.get_logger().info(f'{self.topic_name}: Compressed {len(points)} raw points -> {len(compressed)} gripper commands')

        # Publish commands with timing
        playback_start = time.monotonic()
        progress_providers = progress_providers or []
        FEEDBACK_POLL_DT = 0.05
        FEEDBACK_STALL_TIMEOUT = 2.0

        last_feedback_progress = 0.0
        last_feedback_wall_time = time.monotonic()

        for i, item in enumerate(compressed):
            target_time = item['time']
            while True:
                now = time.monotonic()
                progresses = []
                if sync_feedback and progress_providers:
                    for fn in progress_providers:
                        try:
                            progresses.append(float(fn()))
                        except Exception:
                            continue
                    progresses = [p for p in progresses if p is not None]
                if sync_feedback and progresses:
                    current_progress = max(progresses)
                    if current_progress > last_feedback_progress + 1e-9:
                        last_feedback_progress = current_progress
                        last_feedback_wall_time = now
                    if current_progress + sync_margin >= target_time:
                        break
                    await asyncio.sleep(FEEDBACK_POLL_DT)
                    if (now - last_feedback_wall_time) > FEEDBACK_STALL_TIMEOUT:
                        self.node.get_logger().warn(f'{self.topic_name}: feedback stalled > {FEEDBACK_STALL_TIMEOUT}s, falling back to wall clock')
                        sync_feedback = False
                else:
                    elapsed = now - playback_start
                    remaining = target_time - elapsed
                    if remaining <= 0:
                        break
                    await asyncio.sleep(min(remaining, FEEDBACK_POLL_DT))

            # Publish gripper command
            msg = Float64MultiArray()
            msg.data = [float(item['position'])]
            self.publisher.publish(msg)

        self.node.get_logger().info(f'{self.topic_name}: Gripper command sequence completed (published {len(compressed)} commands)')
        return True


class TrajectoryPlayer(Node):
    def __init__(self, sync_feedback: bool = False, sync_margin: float = 0.0) -> None:
        super().__init__('play_joint_trajectory')
        self._trajectory_clients = {}
        self._gripper_clients = {}
        # Spin rclpy in background so ActionClient futures progress while asyncio runs.
        self._executor = rclpy.executors.MultiThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        self._sync_feedback = sync_feedback
        self._sync_margin = sync_margin

    def shutdown(self) -> None:
        try:
            self._executor.remove_node(self)
        except Exception:
            pass
        self.destroy_node()
        # Executor spin thread will exit once no nodes + shutdown called.
        # We do not join if daemon thread to avoid blocking if already stopped.

    def add_trajectory_client(self, name: str, action_name: str) -> None:
        self._trajectory_clients[name] = _TrajectoryActionClient(action_name, self)

    def add_trajectory_publisher(self, name: str, topic_name: str) -> None:
        self._trajectory_clients[name] = _TrajectoryTopicPublisher(topic_name, self)

    def add_gripper_client(self, name: str, action_name: str) -> None:
        self._gripper_clients[name] = _GripperActionClient(action_name, self)

    def add_gripper_publisher(self, name: str, topic_name: str) -> None:
        self._gripper_clients[name] = _GripperTopicPublisher(topic_name, self)

    async def play_all_joints_async(self, joint_names: List[str], points: List[Dict], rate_scale: float = 1.0) -> None:
        """Play trajectory using all available action clients simultaneously."""
        if not self._trajectory_clients and not self._gripper_clients:
            self.get_logger().error('No action clients configured')
            return

        # Auto-group joints by name patterns
        left_arm_joints = [j for j in joint_names if j.startswith('openarmx_left_joint')]
        right_arm_joints = [j for j in joint_names if j.startswith('openarmx_right_joint')]
        left_gripper_joints = [j for j in joint_names if j.startswith('openarmx_left_finger')]
        right_gripper_joints = [j for j in joint_names if j.startswith('openarmx_right_finger')]

        tasks = []

        # Match controllers by suffix pattern: any key ending with _left_arm, _right_arm, etc.
        for key, client in self._trajectory_clients.items():
            if key.endswith('_left_arm') or key == 'left_arm':
                if left_arm_joints:
                    names, pts = filter_joints(joint_names, points, left_arm_joints)
                    if names:
                        tasks.append((key, client.send_trajectory_async(names, pts, rate_scale)))
            elif key.endswith('_right_arm') or key == 'right_arm':
                if right_arm_joints:
                    names, pts = filter_joints(joint_names, points, right_arm_joints)
                    if names:
                        tasks.append((key, client.send_trajectory_async(names, pts, rate_scale)))

        progress_providers = []
        if self._sync_feedback:
            for tc in self._trajectory_clients.values():
                progress_providers.append(tc.get_latest_feedback_time)

        for key, client in self._gripper_clients.items():
            if key.endswith('_left_gripper') or key == 'left_gripper':
                if left_gripper_joints:
                    names, pts = filter_joints(joint_names, points, left_gripper_joints)
                    if names:
                        tasks.append((key, client.send_gripper_commands_async(
                            names, pts, rate_scale,
                            sync_feedback=self._sync_feedback,
                            progress_providers=progress_providers,
                            sync_margin=self._sync_margin)))
            elif key.endswith('_right_gripper') or key == 'right_gripper':
                if right_gripper_joints:
                    names, pts = filter_joints(joint_names, points, right_gripper_joints)
                    if names:
                        tasks.append((key, client.send_gripper_commands_async(
                            names, pts, rate_scale,
                            sync_feedback=self._sync_feedback,
                            progress_providers=progress_providers,
                            sync_margin=self._sync_margin)))

        if not tasks:
            self.get_logger().error('No valid joint groups found for available controllers')
            return

        # Execute all tasks simultaneously
        self.get_logger().info(f'Starting simultaneous execution of {len(tasks)} controllers...')
        results = await asyncio.gather(*[task for _, task in tasks], return_exceptions=True)
        
        # Report results
        for i, (controller_name, _) in enumerate(tasks):
            result = results[i]
            if isinstance(result, Exception):
                self.get_logger().error(f'{controller_name}: Exception - {result}')
            elif result:
                self.get_logger().info(f'{controller_name}: SUCCESS')
            else:
                self.get_logger().error(f'{controller_name}: FAILED')

    def play_single_controller(self, controller_name: str, joint_names: List[str], points: List[Dict], rate_scale: float = 1.0) -> bool:
        """Play trajectory using a single controller."""
        if controller_name in self._trajectory_clients:
            # Run async function
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                return loop.run_until_complete(
                    self._trajectory_clients[controller_name].send_trajectory_async(joint_names, points, rate_scale))
            finally:
                loop.close()
        elif controller_name in self._gripper_clients:
            # Run async function
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                return loop.run_until_complete(
                    self._gripper_clients[controller_name].send_gripper_commands_async(joint_names, points, rate_scale))
            finally:
                loop.close()
        else:
            self.get_logger().error(f'Controller {controller_name} not available')
            return False


def main() -> None:
    parser = argparse.ArgumentParser(description='Play joint trajectory YAML using action interface')
    parser.add_argument('file', help='YAML file recorded by recorder scripts')
    parser.add_argument('--action', help='Single action name (overrides auto-detection)')
    parser.add_argument('--rate-scale', type=float, default=1.0, help='>1.0 plays faster, <1.0 slower')
    parser.add_argument('--joints', nargs='*', help='Filter to specific joints (e.g., --joints left_joint1 left_joint2)')
    parser.add_argument('--left-arm', action='store_true', help='Filter to left arm joints only')
    parser.add_argument('--right-arm', action='store_true', help='Filter to right arm joints only')
    parser.add_argument('--both-arms', action='store_true', help='Filter to both arm joints only')
    parser.add_argument('--all-joints', action='store_true', help='Play all joints using multiple controllers simultaneously')
    parser.add_argument('--sync-feedback', action='store_true', help='Use arm trajectory feedback time to trigger gripper goals')
    parser.add_argument('--sync-margin', type=float, default=0.0, help='Advance gripper goals when feedback_time + margin >= goal time')
    parser.add_argument('--arm-prefix', type=str, nargs='+', default=[''], help='Namespace prefix(es) for target robot(s). Use multiple to broadcast to several robots simultaneously (e.g. --arm-prefix robot1 robot2). Leave empty for single robot without namespace.')
    args = parser.parse_args()

    data = load_yaml(args.file)
    joint_names = data.get('joint_names', [])
    points = data.get('points', [])
    if not joint_names or not points:
        print('Invalid YAML: missing joint_names or points', file=sys.stderr)
        sys.exit(1)

    # Determine target joints
    target_joints = []
    if args.joints:
        target_joints = args.joints
    elif args.left_arm:
        target_joints = [f'openarmx_left_joint{i}' for i in range(1, 8)]
    elif args.right_arm:
        target_joints = [f'openarmx_right_joint{i}' for i in range(1, 8)]
    elif args.both_arms:
        target_joints = [f'openarmx_left_joint{i}' for i in range(1, 8)] + [f'openarmx_right_joint{i}' for i in range(1, 8)]
    elif args.all_joints or not any([args.left_arm, args.right_arm, args.both_arms, args.joints]):
        # Default: use all joints
        target_joints = joint_names
    
    # Filter joints if needed
    if target_joints and target_joints != joint_names:
        print(f"Filtering to joints: {target_joints}")
        joint_names, points = filter_joints(joint_names, points, target_joints)
        if not joint_names:
            print("Error: No joints remaining after filtering", file=sys.stderr)
            sys.exit(1)

    # Initialize ROS 2
    rclpy.init()

    def normalize_prefix(raw: str) -> str:
        p = raw.strip()
        if not p:
            return ''
        if not p.startswith('/'):
            p = '/' + p
        return p.rstrip('/')

    # Deduplicate and normalize prefixes
    prefixes = list(dict.fromkeys(normalize_prefix(p) for p in args.arm_prefix))

    try:
        if args.action:
            # Single-action mode: only makes sense for one prefix
            prefix = prefixes[0] if prefixes else ''
            player = TrajectoryPlayer(sync_feedback=args.sync_feedback, sync_margin=args.sync_margin)
            action_name = args.action
            if prefix and not action_name.startswith(prefix):
                if not action_name.startswith('/'):
                    action_name = '/' + action_name
                action_name = prefix + action_name
            if 'gripper' in args.action:
                player.add_gripper_client('single', action_name)
            else:
                player.add_trajectory_client('single', action_name)
            success = player.play_single_controller('single', joint_names, points, rate_scale=args.rate_scale)
            print("Trajectory execution completed successfully!" if success else "Trajectory execution failed")
            player.shutdown()
        else:
            # Multi-robot broadcast mode: one TrajectoryPlayer with all robots' controllers registered
            player = TrajectoryPlayer(sync_feedback=args.sync_feedback, sync_margin=args.sync_margin)

            for prefix in prefixes:
                tag = prefix.lstrip('/') or 'default'
                player.add_trajectory_client(
                    f'{tag}_left_arm',
                    f'{prefix}/left_joint_trajectory_controller/follow_joint_trajectory')
                player.add_trajectory_client(
                    f'{tag}_right_arm',
                    f'{prefix}/right_joint_trajectory_controller/follow_joint_trajectory')
                if prefix:
                    player.add_gripper_publisher(
                        f'{tag}_left_gripper',
                        f'{prefix}/left_gripper_controller/commands')
                    player.add_gripper_publisher(
                        f'{tag}_right_gripper',
                        f'{prefix}/right_gripper_controller/commands')
                else:
                    player.add_gripper_client(
                        f'{tag}_left_gripper',
                        '/left_gripper_controller/gripper_cmd')
                    player.add_gripper_client(
                        f'{tag}_right_gripper',
                        '/right_gripper_controller/gripper_cmd')

            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(
                    player.play_all_joints_async(joint_names, points, rate_scale=args.rate_scale))
                print("All trajectories completed!")
            finally:
                loop.close()

            player.shutdown()

    except Exception as e:
        print(f"Error during execution: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
