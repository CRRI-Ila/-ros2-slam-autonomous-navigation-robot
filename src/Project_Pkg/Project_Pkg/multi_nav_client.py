#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

def q_from_yaw(yaw_rad: float):
    # planar yaw -> quaternion (x=y=0)
    return (0.0, 0.0, math.sin(yaw_rad/2.0), math.cos(yaw_rad/2.0))

class MultiNavClient(Node):
    def __init__(self):
        super().__init__('multi_nav_client')

        # Params (optional): frame_id and a list of [x, y, yaw_deg]
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('waypoints', None)

        self.frame_id = self.get_parameter('frame_id').value
        wp_param = self.get_parameter('waypoints').value

        # Default demo waypoints if none provided
        self.waypoints = wp_param if wp_param else [
            [4.0, -3.0,   0.0],
        
        ]

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.run()

    def make_pose(self, x: float, y: float, yaw_deg: float) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = self.frame_id
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = 0.0
        qx, qy, qz, qw = q_from_yaw(math.radians(float(yaw_deg)))
        p.pose.orientation.x = qx
        p.pose.orientation.y = qy
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        return p

    def go(self, pose: PoseStamped) -> bool:
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose action server not available')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = pose

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Reached waypoint')
        return True

    def run(self):
        for i, (x, y, yaw) in enumerate(self.waypoints, start=1):
            self.get_logger().info(f'[{i}/{len(self.waypoints)}] -> x={x:.2f}, y={y:.2f}, yaw={yaw:.1f}°')
            pose = self.make_pose(x, y, yaw)
            if not self.go(pose):
                self.get_logger().error(f'Failed at waypoint {i}')
                break

def main():
    rclpy.init()
    node = MultiNavClient()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
