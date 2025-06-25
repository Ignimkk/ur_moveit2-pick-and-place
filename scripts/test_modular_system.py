#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time

class PickPlaceTester(Node):
    def __init__(self):
        super().__init__('pick_place_tester')
        
        # Publishers
        self.pick_goal_pub = self.create_publisher(PoseStamped, '/pick_goal', 10)
        self.place_goal_pub = self.create_publisher(PoseStamped, '/place_goal', 10)
        self.trigger_pub = self.create_publisher(String, '/pick_place_trigger', 10)
        
        # Subscribers
        self.status_sub = self.create_subscription(String, '/pick_place_status', self.status_callback, 10)
        
        self.get_logger().info('Pick Place Tester initialized')
        
    def status_callback(self, msg):
        self.get_logger().info(f'Status: {msg.data}')
        
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
        
    def trigger_pick_and_place(self):
        """Pick and place 시퀀스 트리거"""
        msg = String()
        msg.data = "start_pick_place"
        self.trigger_pub.publish(msg)
        self.get_logger().info('Triggered pick and place sequence')
        
    def trigger_pick_only(self):
        """Pick만 실행"""
        msg = String()
        msg.data = "start_pick"
        self.trigger_pub.publish(msg)
        self.get_logger().info('Triggered pick only')
        
    def trigger_place_only(self):
        """Place만 실행"""
        msg = String()
        msg.data = "start_place"
        self.trigger_pub.publish(msg)
        self.get_logger().info('Triggered place only')

def main(args=None):
    rclpy.init(args=args)
    
    tester = PickPlaceTester()
    
    try:
        # 3초 대기하여 시스템이 준비되도록 함
        tester.get_logger().info('Waiting 3 seconds for system to be ready...')
        time.sleep(3)
        
        # 시나리오 선택 (사용자가 선택할 수 있도록)
        print("\n=== Pick & Place 테스트 시나리오 ===")
        print("1. Pick and Place 시퀀스 (기본)")
        print("2. Pick만 실행")
        print("3. Place만 실행")
        print("4. 개별 테스트")
        
        choice = input("시나리오를 선택하세요 (1-4, 기본값: 1): ").strip()
        if not choice:
            choice = "1"
        
        if choice == "1":
            # Pick and Place 시퀀스 테스트
            tester.get_logger().info('=== Pick and Place 시퀀스 테스트 ===')
            
            # Pick 목표 설정
            tester.get_logger().info('Setting pick goal...')
            tester.send_pick_goal()
            time.sleep(2)
            
            # Place 목표 설정
            tester.get_logger().info('Setting place goal...')
            tester.send_place_goal()
            time.sleep(2)
            
            # Pick and Place 시퀀스 실행
            tester.get_logger().info('Starting pick and place sequence...')
            tester.trigger_pick_and_place()
            
        elif choice == "2":
            # Pick만 테스트
            tester.get_logger().info('=== Pick만 테스트 ===')
            tester.send_pick_goal()
            time.sleep(2)
            tester.trigger_pick_only()
            
        elif choice == "3":
            # Place만 테스트
            tester.get_logger().info('=== Place만 테스트 ===')
            tester.send_place_goal()
            time.sleep(2)
            tester.trigger_place_only()
            
        elif choice == "4":
            # 개별 테스트
            tester.get_logger().info('=== 개별 테스트 ===')
            
            # Pick 테스트
            tester.get_logger().info('--- Pick 테스트 ---')
            tester.send_pick_goal()
            time.sleep(2)
            tester.trigger_pick_only()
            
            # Pick 완료 대기
            tester.get_logger().info('Waiting for pick to complete...')
            time.sleep(30)
            
            # Place 테스트
            tester.get_logger().info('--- Place 테스트 ---')
            tester.send_place_goal()
            time.sleep(2)
            tester.trigger_place_only()
            
        else:
            tester.get_logger().error('Invalid choice')
            return
        
        # 상태 모니터링
        tester.get_logger().info('Monitoring status for 60 seconds...')
        rclpy.spin_once(tester, timeout_sec=60.0)
        
    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted by user')
    except Exception as e:
        tester.get_logger().error(f'Test failed: {e}')
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 