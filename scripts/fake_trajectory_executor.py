#!/usr/bin/env python3

"""
Fake Trajectory Executor - 仿真轨迹执行器
用于无硬件/仿真环境下，接收 FollowJointTrajectory 命令并发布虚拟的关节状态更新
这样 RViz 就能看到机器人的运动，而不需要实际的硬件控制器。
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

import threading
import time


class FakeTrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('fake_trajectory_executor')
        
        # Action server for FollowJointTrajectory
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'follow_joint_trajectory',
            self.execute_trajectory_callback,
            goal_callback=self.goal_callback,
        )
        
        # Joint state publisher
        self._joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Controller state publisher
        self._state_pub = self.create_publisher(
            JointTrajectoryControllerState,
            '/joint_trajectory_controller/state',
            10
        )
        
        self.get_logger().info('Fake Trajectory Executor started!')
        self.get_logger().info('Listening for commands on /follow_joint_trajectory')
        
        # Current joint state (initialize to all zeros)
        self.current_positions = {}
        self.current_velocities = {}
        
    def goal_callback(self, goal_request):
        """Accept all goals"""
        self.get_logger().info('Received trajectory goal')
        return rclpy.action.GoalResponse.ACCEPT
    
    def execute_trajectory_callback(self, goal_handle):
        """Execute the trajectory"""
        trajectory = goal_handle.request.trajectory
        
        self.get_logger().info(
            f'Executing trajectory with {len(trajectory.points)} points'
        )
        
        # Initialize joint names and positions from first point
        joint_names = trajectory.joint_names
        
        try:
            # Execute trajectory by stepping through waypoints
            for i, point in enumerate(trajectory.points):
                # Check if goal is being cancelled
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    return FollowJointTrajectory.Result()
                
                # Update current positions
                for j_name, j_pos in zip(joint_names, point.positions):
                    self.current_positions[j_name] = j_pos
                
                # Update velocities if available
                if point.velocities:
                    for j_name, j_vel in zip(joint_names, point.velocities):
                        self.current_velocities[j_name] = j_vel
                
                # Publish current state
                self.publish_joint_state(joint_names)
                
                # Wait until time_from_start
                if i < len(trajectory.points) - 1:
                    # Calculate wait time
                    current_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
                    next_time = trajectory.points[i + 1].time_from_start.sec + \
                                trajectory.points[i + 1].time_from_start.nanosec / 1e9
                    wait_time = next_time - current_time
                    
                    if wait_time > 0:
                        time.sleep(wait_time * 0.1)  # Speed up 10x for demo
                
                self.get_logger().debug(f'Point {i+1}/{len(trajectory.points)}')
            
            # Final position
            self.publish_joint_state(joint_names)
            
            # Success!
            goal_handle.succeed()
            self.get_logger().info('Trajectory execution succeeded!')
            
            return FollowJointTrajectory.Result()
            
        except Exception as e:
            self.get_logger().error(f'Error executing trajectory: {e}')
            goal_handle.abort()
            return FollowJointTrajectory.Result()
    
    def publish_joint_state(self, joint_names):
        """Publish fake joint state"""
        now = self.get_clock().now()
        
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = ''
        msg.name = joint_names
        
        # Get current positions
        msg.position = [self.current_positions.get(name, 0.0) for name in joint_names]
        
        # Get velocities if available
        if self.current_velocities:
            msg.velocity = [self.current_velocities.get(name, 0.0) for name in joint_names]
        else:
            msg.velocity = [0.0] * len(joint_names)
        
        # Zero effort
        msg.effort = [0.0] * len(joint_names)
        
        self._joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    executor = FakeTrajectoryExecutor()
    
    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
