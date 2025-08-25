# sudo apt install ros-jazzy-tf-transformations

"""
ROS 2 TF (Transform) 브로드캐스터 학습 예제

이 프로그램은 ROS 2에서 좌표계 변환(Transform) 정보를 브로드캐스트하는 방법을 보여줍니다.
주요 학습 내용:
1. tf2_ros.TransformBroadcaster 사용법
2. TransformStamped 메시지 구성
3. 타이머를 이용한 주기적인 Transform 발행
4. 원운동하는 좌표계 구현

실행 방법:
1. ROS 2 환경 설정: source /opt/ros/jazzy/setup.bash
2. 패키지 빌드: colcon build --packages-select my_tf
3. 실행: ros2 run my_tf my_tf_broadcaster
4. 시각화: rviz2에서 TF 표시

작성자: 학습용 예제
날짜: 2024
"""

import math
 
# ROS 2 파이썬 클라이언트 라이브러리
import rclpy
from rclpy.node import Node  # ROS 2 노드 기본 클래스

# 좌표 변환 메시지 타입
from geometry_msgs.msg import TransformStamped
# TF2 라이브러리 (좌표계 변환 관리)
import tf2_ros

class MyTfBroadcaster(Node):
    """
    TF(Transform) 브로드캐스터 클래스
    
    이 클래스는 ROS 2 Node를 상속받아 좌표계 변환 정보를 주기적으로 발행합니다.
    'world' 좌표계를 기준으로 'moving_frame' 좌표계가 반지름 2m인 원 위에서
    움직이는 모습을 시뮬레이션합니다.
    
    주요 기능:
    - TransformBroadcaster를 사용한 TF 발행
    - 타이머 기반 주기적 업데이트 (10Hz)
    - 원운동 궤적 계산 및 적용
    """
    def __init__(self):
        """
        노드 초기화 함수
        
        노드 이름을 설정하고, TF 브로드캐스터와 타이머를 초기화합니다.
        """
        # 부모 클래스(Node) 초기화 - 노드 이름을 'my_tf_broadcaster'로 설정
        super().__init__('my_tf_broadcaster')
        
        # TransformBroadcaster 객체 생성
        # 이 객체를 통해 TF 정보를 ROS 2 네트워크에 브로드캐스트할 수 있습니다
        self.br = tf2_ros.TransformBroadcaster(self)
        
        # 타이머 설정 (0.1초 = 100ms 간격, 즉 10Hz 주파수)
        # timer_callback 함수를 주기적으로 호출합니다
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 원운동을 위한 각도 변수 초기화 (라디안 단위)
        self.t = 0.0
        
        # 노드 시작 로그 출력
        self.get_logger().info('TF Broadcaster 노드가 시작되었습니다. 원운동 시뮬레이션을 시작합니다.')

    def timer_callback(self):
        """
        타이머 콜백 함수 - 주기적으로 호출됨 (10Hz)
        
        이 함수는 0.1초마다 호출되어 원운동하는 좌표계의 위치를 계산하고
        TF 정보를 브로드캐스트합니다.
        """
        # 각도 t를 조금씩 증가 (0.05 라디안씩)
        # 0.1초 간격으로 0.05씩 증가하면 초당 0.5 라디안 = 약 28.6도씩 회전
        self.t += 0.05

        # 원운동 좌표 계산
        # 반지름 2m인 원 위의 점의 좌표를 삼각함수로 계산
        # x = r * cos(θ), y = r * sin(θ)
        x = 2.0 * math.cos(self.t)  # X축 좌표 (범위: -2.0 ~ 2.0)
        y = 2.0 * math.sin(self.t)  # Y축 좌표 (범위: -2.0 ~ 2.0)
        z = 0.0                     # Z축 좌표 (고정값, 평면 운동)

        # TransformStamped 메시지 객체 생성
        # 이 메시지는 한 좌표계에서 다른 좌표계로의 변환 정보를 담습니다
        t = TransformStamped()
        
        # 헤더 정보 설정
        # 현재 시간 스탬프를 메시지에 추가 (동기화를 위해 중요)
        t.header.stamp = self.get_clock().now().to_msg()
        
        # 좌표계 관계 설정
        t.header.frame_id = 'world'      # 부모 좌표계 (기준점)
        t.child_frame_id = 'moving_frame' # 자식 좌표계 (움직이는 좌표계)
        
        # 위치 변환 정보 설정 (Translation)
        # world 좌표계 기준으로 moving_frame의 위치
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        # 회전 변환 정보는 설정하지 않음 (기본값: 회전 없음)
        # t.transform.rotation은 기본적으로 단위 쿼터니언 (0,0,0,1)로 설정됨
        
        # TF 정보를 ROS 2 네트워크에 브로드캐스트
        # 이제 다른 노드들이 world -> moving_frame 변환을 사용할 수 있습니다
        self.br.sendTransform(t)
        
        # 디버깅용 로그 (너무 자주 출력되지 않도록 조건부 출력)
        if int(self.t * 10) % 31 == 0:  # 약 3초마다 출력
            self.get_logger().info(f'위치 업데이트: x={x:.2f}, y={y:.2f}, 각도={self.t:.2f}rad')

def main(args=None):
    """
    메인 함수 - 프로그램 진입점
    
    ROS 2 시스템을 초기화하고 노드를 실행합니다.
    Ctrl+C로 종료할 때까지 계속 실행됩니다.
    
    Args:
        args: 명령행 인수 (기본값: None)
    """
    # ROS 2 시스템 초기화
    # 이 함수를 호출해야 ROS 2 통신이 가능해집니다
    rclpy.init(args=args)
    
    # TF 브로드캐스터 노드 객체 생성
    node = MyTfBroadcaster()
    
    try:
        # 노드 실행 시작 (무한 루프)
        # 콜백 함수들이 호출되고 ROS 2 메시지를 처리합니다
        print("TF 브로드캐스터가 실행 중입니다...")
        print("종료하려면 Ctrl+C를 누르세요.")
        print("rviz2에서 TF 시각화를 확인해보세요!")
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        # Ctrl+C가 눌렸을 때 정상적으로 종료
        print("\n사용자에 의해 종료되었습니다.")
        pass
        
    finally:
        # 정리 작업
        # 노드 리소스 해제
        node.destroy_node()
        # ROS 2 시스템 종료
        rclpy.shutdown()
        print("TF 브로드캐스터가 정상적으로 종료되었습니다.")

# 스크립트가 직접 실행될 때만 main() 함수 호출
# 다른 모듈에서 import될 때는 실행되지 않음
if __name__ == '__main__':
    main()