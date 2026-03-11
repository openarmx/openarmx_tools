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
        future = self.action_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        goal_handle = await future
        
        if goal_handle.accepted:
            self.node.get_logger().info(f'Goal accepted by {self.action_name}!')
            result_future = goal_handle.get_result_async()
            result = await result_future
            self.node.get_logger().info(f'{self.action_name} result: {result.result.error_string}')
            return result.result.error_code == 0
        else:
            self.node.get_logger().error(f'Goal rejected by {self.action_name}')
            return False


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
            goal_handle = await goal_future
            if not goal_handle.accepted:
                self.node.get_logger().error(f'{self.action_name} goal {i} rejected (pos={goal.command.position})')
                return False
            result_futures.append((i, goal.command.position, goal_handle.get_result_async()))

        # Await all results
        for i, pos, rf in result_futures:
            result = await rf
            if not result.result.reached_goal:
                self.node.get_logger().warning(f'{self.action_name} goal {i}: target not reached (pos={pos})')

        self.node.get_logger().info(f'{self.action_name}: Gripper command sequence completed (dispatched {len(compressed)} goals)')
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

    def add_gripper_client(self, name: str, action_name: str) -> None:
        self._gripper_clients[name] = _GripperActionClient(action_name, self)

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

        # Create tasks for each controller
        tasks = []
        
        # Left arm
        if left_arm_joints and 'left_arm' in self._trajectory_clients:
            left_joint_names, left_points = filter_joints(joint_names, points, left_arm_joints)
            if left_joint_names:
                task = self._trajectory_clients['left_arm'].send_trajectory_async(
                    left_joint_names, left_points, rate_scale)
                tasks.append(('left_arm', task))
        
        # Right arm
        if right_arm_joints and 'right_arm' in self._trajectory_clients:
            right_joint_names, right_points = filter_joints(joint_names, points, right_arm_joints)
            if right_joint_names:
                task = self._trajectory_clients['right_arm'].send_trajectory_async(
                    right_joint_names, right_points, rate_scale)
                tasks.append(('right_arm', task))
        
        # Build list of trajectory progress providers (reference arms) for feedback-sync.
        progress_providers = []
        if self._sync_feedback:
            # Use all trajectory clients (left/right arms) as providers; if only one exists, that's fine.
            for tc in self._trajectory_clients.values():
                progress_providers.append(tc.get_latest_feedback_time)

        # Left gripper
        if left_gripper_joints and 'left_gripper' in self._gripper_clients:
            left_gripper_joint_names, left_gripper_points = filter_joints(joint_names, points, left_gripper_joints)
            if left_gripper_joint_names:
                task = self._gripper_clients['left_gripper'].send_gripper_commands_async(
                    left_gripper_joint_names, left_gripper_points, rate_scale,
                    sync_feedback=self._sync_feedback, progress_providers=progress_providers,
                    sync_margin=self._sync_margin)
                tasks.append(('left_gripper', task))
        
        # Right gripper
        if right_gripper_joints and 'right_gripper' in self._gripper_clients:
            right_gripper_joint_names, right_gripper_points = filter_joints(joint_names, points, right_gripper_joints)
            if right_gripper_joint_names:
                task = self._gripper_clients['right_gripper'].send_gripper_commands_async(
                    right_gripper_joint_names, right_gripper_points, rate_scale,
                    sync_feedback=self._sync_feedback, progress_providers=progress_providers,
                    sync_margin=self._sync_margin)
                tasks.append(('right_gripper', task))

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
    
    try:
        player = TrajectoryPlayer(sync_feedback=args.sync_feedback, sync_margin=args.sync_margin)
        
        if args.action:
            # Single controller mode - determine if it's gripper or trajectory
            if 'gripper' in args.action:
                player.add_gripper_client('single', args.action)
            else:
                player.add_trajectory_client('single', args.action)
            success = player.play_single_controller('single', joint_names, points, rate_scale=args.rate_scale)
            if success:
                print("Trajectory execution completed successfully!")
            else:
                print("Trajectory execution failed")
        else:
            # Multi-controller mode
            player.add_trajectory_client('left_arm', '/left_joint_trajectory_controller/follow_joint_trajectory')
            player.add_trajectory_client('right_arm', '/right_joint_trajectory_controller/follow_joint_trajectory')
            player.add_gripper_client('left_gripper', '/left_gripper_controller/gripper_cmd')
            player.add_gripper_client('right_gripper', '/right_gripper_controller/gripper_cmd')
            
            # Run async function
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(player.play_all_joints_async(joint_names, points, rate_scale=args.rate_scale))
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
