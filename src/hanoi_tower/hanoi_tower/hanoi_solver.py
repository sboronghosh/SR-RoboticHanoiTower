import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import PoseStamped, PoseArray

from enum import Enum
from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple
import time


class SolverState(Enum):
    IDLE = 'idle'
    WAITING_FOR_VISION = 'waiting_for_vision'
    PLANNING = 'planning'
    EXECUTING = 'executing'
    WAITING_FOR_MOVE = 'waiting_for_move'
    COMPLETED = 'completed'
    ERROR = 'error'


@dataclass
class Move:
    cube: str
    source: str
    target: str
    step: int


class HanoiSolverNode(Node):

    def __init__(self):
        super().__init__('hanoi_solver')
        self.declare_parameter('hanoi.num_disks', 3)
        self.declare_parameter('hanoi.source_bin', 'A')
        self.declare_parameter('hanoi.auxiliary_bin', 'B')
        self.declare_parameter('hanoi.target_bin', 'C')
        
        self.num_disks = self.get_parameter('hanoi.num_disks').value
        self.source_bin = self.get_parameter('hanoi.source_bin').value
        self.auxiliary_bin = self.get_parameter('hanoi.auxiliary_bin').value
        self.target_bin = self.get_parameter('hanoi.target_bin').value
        
        self.cube_names = ['large', 'medium', 'small'][:self.num_disks]
        
        self.state = SolverState.IDLE
        self.current_move_index = 0
        self.moves: List[Move] = []
        
        self.bin_state: Dict[str, List[str]] = {
            'A': [],
            'B': [],
            'C': []
        }
        
        self.cube_poses: Dict[str, Optional[PoseStamped]] = {
            name: None for name in self.cube_names
        }
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.cube_poses_sub = self.create_subscription(
            PoseArray,
            '/hanoi/cube_poses',
            self.cube_poses_callback,
            10
        )
        
        for cube_name in self.cube_names:
            self.create_subscription(
                PoseStamped,
                f'/hanoi/cube_{cube_name}_pose',
                lambda msg, name=cube_name: self.individual_cube_callback(msg, name),
                10
            )
        
        self.motion_complete_sub = self.create_subscription(
            Bool,
            '/hanoi/motion_complete',
            self.motion_complete_callback,
            10
        )
        
        self.state_pub = self.create_publisher(String, '/hanoi/solver_state', 10)
        self.current_move_pub = self.create_publisher(String, '/hanoi/current_move', 10)
        
        self.move_command_pub = self.create_publisher(String, '/hanoi/move_command', 10)
        
        self.start_srv = self.create_service(
            Trigger,
            '/hanoi/start',
            self.start_callback,
            callback_group=self.callback_group
        )
        
        self.reset_srv = self.create_service(
            Trigger,
            '/hanoi/reset',
            self.reset_callback,
            callback_group=self.callback_group
        )
        
        self.pause_srv = self.create_service(
            SetBool,
            '/hanoi/pause',
            self.pause_callback,
            callback_group=self.callback_group
        )
        
        self.timer = self.create_timer(0.5, self.state_machine_tick)
        
        self.generate_solution()
    
        self.get_logger().info('Hanoi Solver initialized')
        self.get_logger().info(f'Solution has {len(self.moves)} moves')
        self.print_solution()
    
    def generate_solution(self):
        self.moves = []
        self._hanoi_recursive(
            self.num_disks,
            self.source_bin,
            self.target_bin,
            self.auxiliary_bin
        )
        self.bin_state = {'A': [], 'B': [], 'C': []}
        self.bin_state[self.source_bin] = self.cube_names.copy()
    
    def _hanoi_recursive(self, n: int, source: str, target: str, auxiliary: str):
        if n == 0:
            return
        self._hanoi_recursive(n - 1, source, auxiliary, target)
        cube_name = self.cube_names[self.num_disks - n]
        move = Move(
            cube=cube_name,
            source=source,
            target=target,
            step=len(self.moves) + 1
        )
        self.moves.append(move)
        self._hanoi_recursive(n - 1, auxiliary, target, source)
    
    def print_solution(self):
        self.get_logger().info('=== Tower of Hanoi Solution ===')
        for move in self.moves:
            self.get_logger().info(
                f'Step {move.step}: Move {move.cube} from {move.source} to {move.target}'
            )
        self.get_logger().info('==============================')
    
    def cube_poses_callback(self, msg: PoseArray):
        pass
    
    def individual_cube_callback(self, msg: PoseStamped, cube_name: str):
        self.cube_poses[cube_name] = msg
    
    def motion_complete_callback(self, msg: Bool):
        if msg.data and self.state == SolverState.WAITING_FOR_MOVE:
            self.get_logger().info('Motion completed, proceeding to next move')
            self.current_move_index += 1
            self.state = SolverState.EXECUTING
    
    def start_callback(self, request, response):
        if self.state == SolverState.IDLE or self.state == SolverState.COMPLETED:
            self.state = SolverState.WAITING_FOR_VISION
            self.current_move_index = 0
            response.success = True
            response.message = 'Started solving Tower of Hanoi'
            self.get_logger().info('Starting Tower of Hanoi solver')
        else:
            response.success = False
            response.message = f'Cannot start, current state: {self.state.value}'
        return response
    
    def reset_callback(self, request, response):
        self.state = SolverState.IDLE
        self.current_move_index = 0
        self.generate_solution()
        response.success = True
        response.message = 'Solver reset'
        self.get_logger().info('Solver reset')
        return response
    
    def pause_callback(self, request, response):
        if request.data:
            # Pause
            if self.state == SolverState.EXECUTING:
                self.state = SolverState.IDLE
                response.success = True
                response.message = 'Solver paused'
            else:
                response.success = False
                response.message = f'Cannot pause, current state: {self.state.value}'
        else:
            # Resume
            if self.state == SolverState.IDLE and self.current_move_index > 0:
                self.state = SolverState.EXECUTING
                response.success = True
                response.message = 'Solver resumed'
            else:
                response.success = False
                response.message = f'Cannot resume, current state: {self.state.value}'
        return response
    
    def state_machine_tick(self):
        state_msg = String()
        state_msg.data = self.state.value
        self.state_pub.publish(state_msg)
        
        if self.state == SolverState.IDLE:
            pass
        
        elif self.state == SolverState.WAITING_FOR_VISION:
            visible_cubes = sum(1 for p in self.cube_poses.values() if p is not None)
            if visible_cubes >= 1:
                self.get_logger().info(f'Vision ready, {visible_cubes} cubes detected')
                self.state = SolverState.EXECUTING
            else:
                self.get_logger().debug('Waiting for vision to detect cubes...')
        
        elif self.state == SolverState.EXECUTING:
            if self.current_move_index >= len(self.moves):
                self.state = SolverState.COMPLETED
                self.get_logger().info('Tower of Hanoi completed!')
            else:
                current_move = self.moves[self.current_move_index]
                self.execute_move(current_move)
                self.state = SolverState.WAITING_FOR_MOVE
        
        elif self.state == SolverState.WAITING_FOR_MOVE:
            pass
        
        elif self.state == SolverState.COMPLETED:
            pass
        
        elif self.state == SolverState.ERROR:
            self.get_logger().error('Solver in error state')
    
    def execute_move(self, move: Move):
        self.get_logger().info(
            f'Executing Step {move.step}: Move {move.cube} from {move.source} to {move.target}'
        )
        
        move_msg = String()
        move_msg.data = f'{move.step}/{len(self.moves)}: {move.cube} {move.source}->{move.target}'
        self.current_move_pub.publish(move_msg)
        
        command_msg = String()
        command_msg.data = f'{move.cube},{move.source},{move.target}'
        self.move_command_pub.publish(command_msg)
        
        if move.cube in self.bin_state[move.source]:
            self.bin_state[move.source].remove(move.cube)
            self.bin_state[move.target].append(move.cube)
    
    def get_cube_position(self, cube_name: str) -> Optional[Tuple[float, float, float]]:
        pose = self.cube_poses.get(cube_name)
        if pose is None:
            return None
        return (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z
        )


def main(args=None):
    rclpy.init(args=args)
    node = HanoiSolverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
