#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class PickPlaceTester(Node):
    def __init__(self):
        super().__init__('pick_place_tester')
        
        # Publishers (trigger 제거)
        self.pick_goal_pub = self.create_publisher(PoseStamped, '/pick_goal', 10)
        self.place_goal_pub = self.create_publisher(PoseStamped, '/place_goal', 10)
        
        self.get_logger().info('Pick Place Tester initialized')
        
    def send_pick_goal(self, x=0.010, y=0.410, z=0.264):
        """기존 코드와 동일한 pick 위치로 목표 전송"""
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        
        self.pick_goal_pub.publish(pose)
        self.get_logger().info(f'Sent pick goal: x={x}, y={y}, z={z}')
        
    def send_place_goal(self, x=-0.340, y=0.310, z=0.264):
        """기존 코드와 동일한 place 위치로 목표 전송"""
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        
        self.place_goal_pub.publish(pose)
        self.get_logger().info(f'Sent place goal: x={x}, y={y}, z={z}')

def main(args=None):
    rclpy.init(args=args)
    
    tester = PickPlaceTester()
    
    try:
        # 3초 대기하여 시스템이 준비되도록 함
        tester.get_logger().info('Waiting 3 seconds for system to be ready...')
        time.sleep(3)
        
        # 자동으로 Pick and Place 시퀀스 실행
        tester.get_logger().info('=== Auto Pick and Place Sequence ===')
        
        # Pick 목표 설정
        tester.get_logger().info('Setting pick goal...')
        tester.send_pick_goal()
        time.sleep(2)
        
        # Place 목표 설정
        tester.get_logger().info('Setting place goal...')
        tester.send_place_goal()
        time.sleep(1)
        
        tester.get_logger().info('Pick and place goals set - sequence will start automatically')
        
        # 상태 모니터링 (더 긴 시간으로 변경)
        tester.get_logger().info('Monitoring status for 120 seconds...')
        start_time = time.time()
        while time.time() - start_time < 120:
            rclpy.spin_once(tester, timeout_sec=1.0)
            
    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted by user')
    except Exception as e:
        tester.get_logger().error(f'Test failed: {e}')
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 