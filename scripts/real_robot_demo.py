#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
import time

class RealRobotPickPlaceDemo(Node):
    def __init__(self):
        super().__init__('real_robot_pick_place_demo')
        
        # Publishers for pick and place goals
        self.pick_goal_pub = self.create_publisher(PoseStamped, '/pick_goal', 10)
        self.place_goal_pub = self.create_publisher(PoseStamped, '/place_goal', 10)
        
        # Subscribers for robot status monitoring
        self.robot_mode_sub = self.create_subscription(
            RobotMode, 
            '/io_and_status_controller/robot_mode', 
            self.robot_mode_callback, 
            10
        )
        self.safety_mode_sub = self.create_subscription(
            SafetyMode,
            '/io_and_status_controller/safety_mode',
            self.safety_mode_callback,
            10
        )
        
        # Robot status
        self.robot_mode = None
        self.safety_mode = None
        self.robot_ready = False
        
        self.get_logger().info('Real Robot Pick Place Demo initialized')
        self.get_logger().warn('SAFETY WARNING: Ensure robot workspace is clear!')
        
    def robot_mode_callback(self, msg):
        self.robot_mode = msg.mode
        self.check_robot_status()
        
    def safety_mode_callback(self, msg):
        self.safety_mode = msg.mode
        self.check_robot_status()
        
    def check_robot_status(self):
        """로봇 상태 확인 및 준비 상태 판단"""
        if self.robot_mode is not None and self.safety_mode is not None:
            # Robot mode: RUNNING(7), Safety mode: NORMAL(1)
            if self.robot_mode == 7 and self.safety_mode == 1:
                if not self.robot_ready:
                    self.robot_ready = True
                    self.get_logger().info('✅ Robot is READY for operation!')
            else:
                if self.robot_ready:
                    self.robot_ready = False
                    self.get_logger().warn(f'⚠️  Robot not ready - Mode: {self.robot_mode}, Safety: {self.safety_mode}')
        
    def send_pick_goal(self, x=0.3, y=0.2, z=0.3):
        """안전한 Pick 위치로 목표 전송 (기본값: 더 보수적인 위치)"""
        if not self.robot_ready:
            self.get_logger().error('❌ Robot not ready! Check robot status.')
            return False
            
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # 안전한 방향 (엔드이펙터가 아래를 향하도록)
        pose.pose.orientation.w = 0.707
        pose.pose.orientation.x = 0.707
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        
        self.pick_goal_pub.publish(pose)
        self.get_logger().info(f'🎯 Pick goal sent: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        return True
        
    def send_place_goal(self, x=-0.2, y=0.3, z=0.3):
        """안전한 Place 위치로 목표 전송 (기본값: 더 보수적인 위치)"""
        if not self.robot_ready:
            self.get_logger().error('❌ Robot not ready! Check robot status.')
            return False
            
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # 안전한 방향
        pose.pose.orientation.w = 0.707
        pose.pose.orientation.x = 0.707
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        
        self.place_goal_pub.publish(pose)
        self.get_logger().info(f'🎯 Place goal sent: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        return True

def main(args=None):
    rclpy.init(args=args)
    
    demo = RealRobotPickPlaceDemo()
    
    try:
        # 로봇 상태 확인을 위한 대기
        demo.get_logger().info('⏳ Waiting for robot status...')
        start_time = time.time()
        
        while not demo.robot_ready and (time.time() - start_time) < 30.0:
            rclpy.spin_once(demo, timeout_sec=1.0)
            
        if not demo.robot_ready:
            demo.get_logger().error('❌ Robot not ready after 30 seconds. Check robot connection and status.')
            return
            
        # 사용자 확인
        demo.get_logger().info('🚨 SAFETY CHECK: Is the robot workspace clear? (Press Enter to continue or Ctrl+C to abort)')
        input()
        
        # 안전한 데모 시퀀스 실행
        demo.get_logger().info('🤖 Starting SAFE pick and place demo...')
        
        # Step 1: Pick goal
        demo.get_logger().info('📍 Step 1: Sending pick goal...')
        if demo.send_pick_goal():
            time.sleep(3)  # 동작 완료 대기
            
            # Step 2: Place goal  
            demo.get_logger().info('📍 Step 2: Sending place goal...')
            if demo.send_place_goal():
                time.sleep(3)
                
                demo.get_logger().info('✅ Demo completed successfully!')
            else:
                demo.get_logger().error('❌ Failed to send place goal')
        else:
            demo.get_logger().error('❌ Failed to send pick goal')
            
        # 상태 모니터링 계속
        demo.get_logger().info('📊 Monitoring robot status... (Ctrl+C to exit)')
        while True:
            rclpy.spin_once(demo, timeout_sec=1.0)
            
    except KeyboardInterrupt:
        demo.get_logger().info('🛑 Demo stopped by user')
    except Exception as e:
        demo.get_logger().error(f'❌ Demo failed: {e}')
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 