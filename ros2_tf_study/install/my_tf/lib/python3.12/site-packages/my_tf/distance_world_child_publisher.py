"""
ROS 2 TF (Transform) 조회 및 거리 계산 학습 예제

이 프로그램은 ROS 2에서 TF 정보를 조회하여 두 좌표계 간의 거리를 계산하고
그 결과를 토픽으로 발행하는 방법을 보여줍니다.

주요 학습 내용:
1. tf2_ros.Buffer와 TransformListener 사용법
2. lookup_transform()을 통한 TF 정보 조회
3. 3차원 유클리드 거리 계산 공식
4. ROS 2 Publisher를 통한 데이터 발행
5. TF 조회 시 예외 처리의 중요성
6. 실시간 TF 모니터링 시스템 구현

전제 조건:
- my_tf_1 노드가 실행되어 world -> moving_frame TF를 발행 중이어야 함
- child_frame 노드가 실행되어 moving_frame -> child_frame TF를 발행 중이어야 함

실행 방법:
1. 터미널 1: ros2 run my_tf my_tf_1
2. 터미널 2: ros2 run my_tf child_frame  
3. 터미널 3: ros2 run my_tf distance_world_child_publisher
4. 결과 확인: ros2 topic echo /distance_world_child

수학적 배경:
- 유클리드 거리: d = √(x² + y² + z²)
- TF 체인: world -> moving_frame -> child_frame
- world에서 child_frame까지의 최종 변환을 자동 계산

작성자: 학습용 예제
날짜: 2024
"""

# 수학 연산을 위한 라이브러리 (제곱근, 거듭제곱 등)
import math
# ROS 2 파이썬 클라이언트 라이브러리
import rclpy
from rclpy.node import Node  # ROS 2 노드 기본 클래스

# TF2 라이브러리 (좌표계 변환 조회용)
import tf2_ros
# 표준 메시지 타입 (32비트 부동소수점)
from std_msgs.msg import Float32

class DistanceWorldChildPublisher(Node):
    """
    TF 정보를 조회하여 거리를 계산하고 발행하는 클래스
    
    이 클래스는 ROS 2의 TF 시스템을 사용하여 'world' 좌표계에서 'child_frame' 
    좌표계까지의 거리를 실시간으로 계산하고, 그 결과를 토픽으로 발행합니다.
    
    주요 기능:
    - TF Buffer를 통한 좌표계 변환 정보 저장 및 관리
    - TransformListener를 통한 실시간 TF 정보 수신
    - lookup_transform()을 사용한 특정 좌표계 간 변환 조회
    - 3차원 유클리드 거리 계산
    - Float32 메시지를 통한 거리 정보 발행
    
    TF 체인 예시:
    world (원점) -> moving_frame (원운동) -> child_frame (자식)
    최종적으로 world에서 child_frame까지의 직선 거리를 계산
    """
    def __init__(self):
        """
        노드 초기화 함수
        
        TF 조회를 위한 Buffer와 Listener, 거리 발행을 위한 Publisher,
        그리고 주기적 실행을 위한 Timer를 설정합니다.
        """
        # 부모 클래스(Node) 초기화 - 노드 이름을 'distance_world_child_publisher'로 설정
        super().__init__('distance_world_child_publisher')
        
        # === TF 조회 시스템 설정 ===
        # TF Buffer: 과거의 TF 정보를 일정 시간 동안 저장하는 메모리 버퍼
        # 기본적으로 10초간의 TF 히스토리를 보관합니다
        self.tf_buffer = tf2_ros.Buffer()
        
        # TF Listener: 네트워크에서 TF 정보를 실시간으로 수신하여 Buffer에 저장
        # 모든 /tf 토픽의 데이터를 자동으로 수집하고 관리합니다
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # === Publisher 설정 ===
        # Float32 타입의 메시지로 거리 정보를 'distance_world_child' 토픽에 발행
        # 큐 크기: 10 (네트워크 지연을 대비한 메시지 버퍼링)
        self.dist_pub = self.create_publisher(Float32, 'distance_world_child', 10)
        
        # === 타이머 설정 ===
        # 0.1초(100ms) 간격으로 timer_callback 함수를 호출 (10Hz 주파수)
        # 실시간 모니터링을 위한 적절한 업데이트 주기
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 노드 시작 로그 출력
        self.get_logger().info('거리 계산 노드가 시작되었습니다.')
        self.get_logger().info('world -> child_frame 거리를 실시간으로 계산합니다.')
        self.get_logger().info('결과는 /distance_world_child 토픽에 발행됩니다.')

    def timer_callback(self):
        """
        타이머 콜백 함수 - 주기적으로 TF 조회 및 거리 계산 (10Hz)
        
        이 함수는 0.1초마다 호출되어 다음 작업을 수행합니다:
        1. TF Buffer에서 world -> child_frame 변환 조회
        2. 변환 정보에서 위치 좌표 추출 
        3. 3차원 유클리드 거리 계산
        4. 계산된 거리를 토픽으로 발행
        """
        try:
            # === TF 변환 조회 ===
            # lookup_transform()을 사용하여 두 좌표계 간의 변환 관계 조회
            # 파라미터: (목표 프레임, 소스 프레임, 시간)
            transform = self.tf_buffer.lookup_transform(
                'world',         # 목표 프레임 (기준점, 거리 측정의 출발점)
                'child_frame',   # 소스 프레임 (측정 대상, 거리 측정의 도착점) 
                rclpy.time.Time()  # 시간 (빈 Time()은 "가장 최근" 의미)
            )

            # === 좌표 정보 추출 ===
            # TransformStamped 메시지에서 translation (위치) 정보 추출
            # world 좌표계 기준으로 child_frame의 절대 위치
            x = transform.transform.translation.x  # X축 좌표 (미터 단위)
            y = transform.transform.translation.y  # Y축 좌표 (미터 단위)
            z = transform.transform.translation.z  # Z축 좌표 (미터 단위)

            # === 3차원 유클리드 거리 계산 ===
            # 공식: d = √(x² + y² + z²)
            # 원점 (0,0,0)에서 (x,y,z)까지의 직선 거리
            distance = math.sqrt(x**2 + y**2 + z**2)

            # === 메시지 발행 ===
            # Float32 메시지 객체 생성 및 데이터 설정
            msg = Float32()
            msg.data = distance  # 계산된 거리 값 (미터 단위)
            
            # 'distance_world_child' 토픽으로 거리 정보 발행
            self.dist_pub.publish(msg)
            
            # 디버깅용 로그 (너무 자주 출력되지 않도록 조건부)
            # 약 1초마다 한 번씩 로그 출력 (10Hz 중 1번)
            if abs(distance - round(distance, 1)) < 0.01:  # 거리가 소수점 첫째자리에서 반올림될 때
                self.get_logger().info(f'거리: {distance:.3f}m (좌표: x={x:.2f}, y={y:.2f}, z={z:.2f})')

        except Exception as e:
            # === 예외 처리 ===
            # TF 조회 실패 시 발생할 수 있는 상황들:
            # 1. 아직 필요한 TF가 발행되지 않음 (노드가 시작되지 않음)
            # 2. 네트워크 지연으로 인한 TF 데이터 부족
            # 3. 좌표계 이름 오타 또는 존재하지 않는 프레임
            # 4. TF 체인이 끊어진 경우 (중간 프레임 누락)
            
            # 경고 레벨 로그로 오류 상황 알림 (에러는 아님, 일시적일 수 있음)
            self.get_logger().warn(f'TF 변환 조회 실패: {e}')
            self.get_logger().warn('다음을 확인하세요:')
            self.get_logger().warn('1. my_tf_1 노드가 실행 중인지 (world -> moving_frame)')
            self.get_logger().warn('2. child_frame 노드가 실행 중인지 (moving_frame -> child_frame)')

def main(args=None):
    """
    메인 함수 - 프로그램 진입점
    
    거리 계산 노드를 초기화하고 실행합니다.
    다른 TF 발행 노드들이 먼저 실행되어야 정상 동작합니다.
    
    Args:
        args: 명령행 인수 (기본값: None)
    """
    # ROS 2 시스템 초기화
    rclpy.init(args=args)
    
    # 거리 계산 노드 객체 생성
    node = DistanceWorldChildPublisher()
    
    try:
        # 노드 실행 시작 (무한 루프)
        # TF 정보를 실시간으로 모니터링하며 거리를 계산합니다
        print("=== 거리 계산 노드 실행 중 ===")
        print("전제 조건:")
        print("1. my_tf_1 노드가 실행 중이어야 함")
        print("2. child_frame 노드가 실행 중이어야 함")
        print("")
        print("결과 확인: ros2 topic echo /distance_world_child")
        print("종료하려면 Ctrl+C를 누르세요.")
        print("=====================================")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        # Ctrl+C가 눌렸을 때 정상적으로 종료
        print("\n사용자에 의해 종료되었습니다.")
        
    finally:
        # 정리 작업
        # 노드 리소스 해제
        node.destroy_node()
        # ROS 2 시스템 종료
        rclpy.shutdown()
        print("거리 계산 노드가 정상적으로 종료되었습니다.")

# 스크립트가 직접 실행될 때만 main() 함수 호출
# 다른 모듈에서 import될 때는 실행되지 않음
if __name__ == '__main__':
    main()