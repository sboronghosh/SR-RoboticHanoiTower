import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from linkattacher_msgs.srv import AttachLink, DetachLink
from control_msgs.action import FollowJointTrajectory
from xarm_msgs.srv import PlanPose, PlanExec, PlanSingleStraight, PlanJoint

import math
from typing import Dict, Optional, List
from dataclasses import dataclass
from enum import Enum
import time
from builtin_interfaces.msg import Duration


class MotionState(Enum):
    IDLE = 'idle'
    MOVING_TO_PREGRASP = 'moving_to_pregrasp'
    LOWERING_TO_GRASP = 'lowering_to_grasp'
    CLOSING_GRIPPER = 'closing_gripper'
    LIFTING = 'lifting'
    MOVING_TO_PLACE = 'moving_to_place'
    LOWERING_TO_PLACE = 'lowering_to_place'
    OPENING_GRIPPER = 'opening_gripper'
    RETREATING = 'retreating'
    COMPLETED = 'completed'
    ERROR = 'error'


@dataclass
class BinPosition:
    x: float
    y: float
    z: float

class MotionExecutorNode(Node):   
    def __init__(self):
        super().__init__('motion_executor')
        
        self.declare_parameter('bins.A.x', 0.05)
        self.declare_parameter('bins.A.y', -0.65)
        self.declare_parameter('bins.A.z', 1.025)
        self.declare_parameter('bins.B.x', 0.20)
        self.declare_parameter('bins.B.y', -0.65)
        self.declare_parameter('bins.B.z', 1.025)
        self.declare_parameter('bins.C.x', 0.35)
        self.declare_parameter('bins.C.y', -0.65)
        self.declare_parameter('bins.C.z', 1.025)
        
        self.declare_parameter('motion.approach_height', 0.10)
        self.declare_parameter('motion.grasp_height', 0.0)
        self.declare_parameter('motion.lift_height', 0.08)
        self.declare_parameter('motion.gripper_settle_time', 0.3)
        
        self.declare_parameter('cubes.small.size', 0.04)
        self.declare_parameter('cubes.medium.size', 0.05)
        self.declare_parameter('cubes.large.size', 0.06)
        
        self.bins: Dict[str, BinPosition] = {}
        for bin_name in ['A', 'B', 'C']:
            self.bins[bin_name] = BinPosition(
                x=self.get_parameter(f'bins.{bin_name}.x').value,
                y=self.get_parameter(f'bins.{bin_name}.y').value,
                z=self.get_parameter(f'bins.{bin_name}.z').value,
            )
        
        self.approach_height = self.get_parameter('motion.approach_height').value
        self.grasp_height = self.get_parameter('motion.grasp_height').value
        self.lift_height = self.get_parameter('motion.lift_height').value
        self.gripper_settle_time = self.get_parameter('motion.gripper_settle_time').value
        
        self.cube_sizes = {
            'small': self.get_parameter('cubes.small.size').value,
            'medium': self.get_parameter('cubes.medium.size').value,
            'large': self.get_parameter('cubes.large.size').value,
        }

        self.state = MotionState.IDLE
        self.current_cube: Optional[str] = None
        self.current_source: Optional[str] = None
        self.current_target: Optional[str] = None
        self.attached_cube: Optional[str] = None
        self.bin_contents: Dict[str, List[str]] = {
            'A': ['large', 'medium', 'small'],
            'B': [],
            'C': []
        }
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.pose_plan_client = self.create_client(
            PlanPose,
            '/xarm_pose_plan',
            callback_group=self.callback_group
        )
        self.exec_plan_client = self.create_client(
            PlanExec,
            '/xarm_exec_plan',
            callback_group=self.callback_group
        )
        self.straight_plan_client = self.create_client(
            PlanSingleStraight,
            '/xarm_straight_plan',
            callback_group=self.callback_group
        )
        
        self.gripper_action_client = ActionClient(self, FollowJointTrajectory, '/xarm_gripper_traj_controller/follow_joint_trajectory', callback_group=self.callback_group)
        
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK', callback_group=self.callback_group)
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK', callback_group=self.callback_group)

        self.move_cmd_sub = self.create_subscription(
            String, '/hanoi/move_command', self.move_command_callback, 10
        )
        
        self.motion_complete_pub = self.create_publisher(Bool, '/hanoi/motion_complete', 10)
        self.state_pub = self.create_publisher(String, '/hanoi/motion_state', 10)
        
        self.get_logger().info('Waiting for xarm_planner services...')
        services_ready = True
        
        if not self.pose_plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('/xarm_pose_plan service not available')
            services_ready = False
        if not self.exec_plan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('/xarm_exec_plan service not available')
            services_ready = False
            
        if services_ready:
            self.get_logger().info('xarm_planner services connected')
        else:
            self.get_logger().warn('Some services not available - motion will fail')
        
        self.timer = self.create_timer(0.1, self.publish_state)
        
        self.get_logger().info('Motion Executor initialized')
        self.get_logger().info(f'Bin contents: {self.bin_contents}')
        
        self.startup_timer = self.create_timer(5.0, self.startup_gripper_open)
    
    def startup_gripper_open(self):
        self.startup_timer.cancel()
        if self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Opening gripper at startup...')
            self.set_gripper(closed=False)
        else:
            self.get_logger().warn('Gripper action server not ready at startup')
    
    def move_command_callback(self, msg: String):

        if self.state != MotionState.IDLE:
            self.get_logger().warn(f'Busy ({self.state.value}), ignoring')
            return
        
        try:
            parts = msg.data.split(',')
            self.current_cube = parts[0].strip()
            self.current_source = parts[1].strip()
            self.current_target = parts[2].strip()
            
            self.get_logger().info(f'=== Move: {self.current_cube} {self.current_source}→{self.current_target} ===')
            self.execute_pick_and_place()
            
        except Exception as e:
            self.get_logger().error(f'Invalid command: {e}')
    
    def execute_pick_and_place(self):
        try:
            src = self.bins[self.current_source]
            tgt = self.bins[self.current_target]
            cube_size = self.cube_sizes[self.current_cube]
            
            src_stack = self.get_stack_height(self.current_source, exclude_top=True)
            cube_z = src.z + src_stack + cube_size / 2
            
            tgt_stack = self.get_stack_height(self.current_target)
            
            self.state = MotionState.MOVING_TO_PREGRASP
            pose = self.create_pose(src.x, src.y, cube_z + self.approach_height)
            self.get_logger().info('Step 1: Pregrasp')
            if not self.move_to_pose(pose):
                return self.fail()
            
            self.state = MotionState.LOWERING_TO_GRASP
            pose = self.create_pose(src.x, src.y, cube_z + self.grasp_height)
            self.get_logger().info('Step 2: Lower to grasp')
            if not self.move_to_pose(pose, cartesian=True):
                return self.fail()
            
            self.state = MotionState.CLOSING_GRIPPER
            self.get_logger().info('Step 3: Gripper close')
            if not self.set_gripper(closed=True, cube_name=self.current_cube):
                return self.fail()
            
            if self.current_cube in self.bin_contents[self.current_source]:
                self.bin_contents[self.current_source].remove(self.current_cube)
            
            time.sleep(1.0)
            
            self.state = MotionState.LIFTING
            pose = self.create_pose(src.x, src.y, cube_z + self.approach_height)
            self.get_logger().info('Step 4: Lift')
            if not self.move_to_pose(pose, cartesian=True):
                return self.fail()
            
            self.state = MotionState.MOVING_TO_PLACE
            tgt_z = tgt.z + tgt_stack + cube_size / 2 + self.lift_height
            pose = self.create_pose(tgt.x, tgt.y, tgt_z)
            self.get_logger().info('Step 5: Move to target')
            if not self.move_to_pose(pose):
                return self.fail()
            
            self.state = MotionState.LOWERING_TO_PLACE
            pose = self.create_pose(tgt.x, tgt.y, tgt.z + tgt_stack + cube_size / 2 + self.grasp_height)
            self.get_logger().info('Step 6: Lower to place')
            if not self.move_to_pose(pose, cartesian=True):
                return self.fail()
            
            self.state = MotionState.OPENING_GRIPPER
            self.get_logger().info('Step 7: Gripper open')
            if not self.set_gripper(closed=False):
                return self.fail()
            
            self.bin_contents[self.current_target].append(self.current_cube)
            
            self.state = MotionState.RETREATING
            pose = self.create_pose(tgt.x, tgt.y, tgt.z + tgt_stack + cube_size + self.approach_height)
            self.get_logger().info('Step 8: Retreat')
            if not self.move_to_pose(pose, cartesian=True):
                return self.fail()
            
            self.state = MotionState.COMPLETED
            self.get_logger().info(f'=== Complete: {self.current_cube} now in {self.current_target} ===')
            
            msg = Bool()
            msg.data = True
            self.motion_complete_pub.publish(msg)
            
            self.current_cube = None
            self.current_source = None
            self.current_target = None
            self.state = MotionState.IDLE
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            self.fail()
    
    def fail(self):
        self.state = MotionState.ERROR
        self.get_logger().error('Motion failed')
        msg = Bool()
        msg.data = False
        self.motion_complete_pub.publish(msg)
    
    def get_stack_height(self, bin_name: str, exclude_top: bool = False) -> float:
        contents = self.bin_contents[bin_name]
        if exclude_top and contents:
            contents = contents[:-1]
        return sum(self.cube_sizes[c] for c in contents)
    
    def create_pose(self, x: float, y: float, z: float) -> Pose:
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        return pose
    
    def wait_for_future(self, future, timeout_sec: float = 10.0) -> bool:
        start = time.time()
        while not future.done():
            if time.time() - start > timeout_sec:
                return False
            time.sleep(0.05)
        return True
    
    def move_to_pose(self, target: Pose, cartesian: bool = False) -> bool:
        if cartesian:
            if not self.straight_plan_client.service_is_ready():
                self.get_logger().error('xarm_straight_plan not ready')
                return False
        else:
            if not self.pose_plan_client.service_is_ready():
                self.get_logger().error('xarm_pose_plan not ready')
                return False
        
        self.get_logger().debug(f'Planning to ({target.position.x:.3f}, {target.position.y:.3f}, {target.position.z:.3f})')
        
        if cartesian:
            req = PlanSingleStraight.Request()
            req.target = target
            future = self.straight_plan_client.call_async(req)
        else:
            req = PlanPose.Request()
            req.target = target
            future = self.pose_plan_client.call_async(req)
        
        if not self.wait_for_future(future, timeout_sec=10.0):
            self.get_logger().error('Planning call timed out')
            return False
        
        if future.result() is None:
            self.get_logger().error('Planning call failed')
            return False
        
        if not future.result().success:
            self.get_logger().error('Planning failed (IK or path not found)')
            return False
        
        self.get_logger().debug('Planning succeeded')
        
        if not self.exec_plan_client.service_is_ready():
            self.get_logger().error('xarm_exec_plan not ready')
            return False
        
        exec_req = PlanExec.Request()
        exec_req.wait = True 
        exec_future = self.exec_plan_client.call_async(exec_req)
        
        if not self.wait_for_future(exec_future, timeout_sec=60.0):
            self.get_logger().error('Execution timed out')
            return False
        
        if exec_future.result() is None:
            self.get_logger().error('Execution call failed')
            return False
        
        if not exec_future.result().success:
            self.get_logger().error('Execution failed')
            return False
        
        self.get_logger().debug('Execution succeeded')
        return True
    
    def attach_object(self, cube_name: str) -> bool:
        self.get_logger().info(f'Attempting to attach {cube_name}...')
        
        service_ready = self.attach_client.service_is_ready()
        self.get_logger().info(f'AttachLink service ready: {service_ready}')
        
        if not service_ready:
            self.get_logger().info('Waiting for AttachLink service...')
            if not self.attach_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().warn(f'LinkAttacher not available after 3s - relying on gripper contact only')
                return True 

        req = AttachLink.Request()
        req.model1_name = 'UF_ROBOT'
        req.link1_name = 'link6'
        req.model2_name = f'cube_{cube_name}'
        req.link2_name = 'cube_link'
        
        self.get_logger().info(f'Calling ATTACHLINK: UF_ROBOT::link6 -> cube_{cube_name}::cube_link')
        future = self.attach_client.call_async(req)
        
        if not self.wait_for_future(future, timeout_sec=5.0):
            self.get_logger().error('Attach service call timed out')
            return False
        
        result = future.result()
        if result is not None:
            self.get_logger().info(f'Attach result: {result.message}')
            return True
        else:
            self.get_logger().error('Attach service call failed - no result')
            return False

    def detach_object(self, cube_name: str) -> bool:
        if not self.detach_client.wait_for_service(timeout_sec=1.0):
             self.get_logger().warn('LinkAttacher service not available - proceeding anyway')
             return True
             
        req = DetachLink.Request()
        req.model1_name = 'UF_ROBOT'
        req.link1_name = 'link6'
        req.model2_name = f'cube_{cube_name}'
        req.link2_name = 'cube_link'
        
        self.get_logger().info(f'Calling DETACHLINK: UF_ROBOT::link6 -> cube_{cube_name}::cube_link')
        future = self.detach_client.call_async(req)
        
        if not self.wait_for_future(future, timeout_sec=2.0):
            self.get_logger().warn('Detach service call timed out')
            return False
            
        result = future.result()
        if result is not None:
            self.get_logger().info(f'Detach result: {result.message}')
            return True
        return False

    def set_gripper(self, closed: bool, cube_name: str = None) -> bool:
        if closed and cube_name and cube_name in self.cube_sizes:
            cube_size = self.cube_sizes[cube_name]
            grip_inwards = 0.006
            required_closing = cube_size - grip_inwards
            max_gap = 0.085    
            position = max(0.0, min(0.85, 0.85 * (1 - required_closing / max_gap)))
            self.get_logger().info(f'Gripper CLOSING for {cube_name} (size={cube_size*1000:.0f}mm, target={position:.3f})')
        elif closed:
            position = 0.85
            self.get_logger().info(f'Gripper CLOSING (target={position})')
        else:
            position = 0.0
            if cube_name:
                 self.detach_object(cube_name)
            self.get_logger().info(f'Gripper OPENING (target={position})')
        
        duration_sec = 4.0
        
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Gripper action server not available')
            return False
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['drive_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        goal.trajectory.points = [point]
        
        send_goal_future = self.gripper_action_client.send_goal_async(goal)
        
        if not self.wait_for_future(send_goal_future, timeout_sec=5.0):
            self.get_logger().error('Gripper goal send timed out')
            return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            return False
        
        result_future = goal_handle.get_result_async()
        if not self.wait_for_future(result_future, timeout_sec=duration_sec + 30.0):
            self.get_logger().error('Gripper execution timed out')
            return False
        
        result = result_future.result()
        if result.result.error_code != 0:
            self.get_logger().error(f'Gripper execution failed: {result.result.error_string}')
            return False
        
        time.sleep(self.gripper_settle_time)
        
        if closed and cube_name:
            if self.attach_object(cube_name):
                 self.attached_cube = cube_name
            else:
                 self.get_logger().warn(f'Could not attach {cube_name} - continuing with gripper contact')
        elif not closed and self.attached_cube:
            if self.detach_object(self.attached_cube):
                self.attached_cube = None
            else:
                self.get_logger().warn(f'Could not detach {self.attached_cube}')
            
        self.get_logger().info(f'Gripper {"CLOSED" if closed else "OPEN"} successfully')
        return True
    
    def publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotionExecutorNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()