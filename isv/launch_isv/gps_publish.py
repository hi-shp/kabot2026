import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import os

class SimpleGpsPublisher(Node):
    def __init__(self):
        super().__init__('simple_gps_publisher')
        
        # QoS를 Reliable로 설정하여 통신 충돌 해결
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 시작 좌표 설정 (리스트 형태: [latitude, longitude])
        self.current_pos = [35.2316341, 129.0824295]
        
        # 터미널 화면 초기화
        os.system('clear')

    def timer_callback(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        
        # 0.1초마다 북쪽으로 약 1.1cm 이동 (리스트 인덱스 0: lat 업데이트)
        self.current_pos[0] += 0.00000
        
        msg.latitude = self.current_pos[0]
        msg.longitude = self.current_pos[1]
        msg.status.status = 2  # RTK Fix 상태 강제
        
        self.publisher_.publish(msg)
        
        # 기존 프린트 형식 유지
        print("\033[H", end="") 
        print("="*45)
        print(f" [GPS 퍼블리시]")
        print(f" 현재 위치: Lat {self.current_pos[0]:.8f}, Lon {self.current_pos[1]:.8f} ")
        print("="*45)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleGpsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()