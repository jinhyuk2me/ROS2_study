"""
ROS 2 TF 궁적(Trail) 시각화 학습 예제

이 프로그램은 ROS 2에서 TF 정보를 실시간으로 조회하여 특정 좌표계의 이동 궁적을
rviz2에서 시각화할 수 있는 Marker 메시지를 발행하는 방법을 보여줍니다.

주요 학습 내용:
1. visualization_msgs/Marker 메시지 사용법
2. LINE_STRIP 마커 타입을 이용한 궁적 그리기
3. 실시간 TF 조회와 위치 데이터 누적
4. rviz2 시각화 시스템 활용
5. Marker 속성 설정 (색상, 크기, 수명 등)
6. 동적 데이터 수집과 시각화의 연동

시각화 결과:
- child_frame이 원운동하면서 남기는 궁적을 빨간색 선으로 표시
- 실시간으로 부드럽게 연결되는 경로 선 그리기
- rviz2에서 /frame_trail_marker 토픽으로 확인 가능

전제 조건:
- my_tf_1 노드가 실행되어 world -> moving_frame TF를 발행 중이어야 함
- child_frame 노드가 실행되어 moving_frame -> child_frame TF를 발행 중이어야 함

실행 방법:
1. 터미널 1: ros2 run my_tf my_tf_1
2. 터미널 2: ros2 run my_tf child_frame
3. 터미널 3: ros2 run my_tf frame_trail_publisher_marker
4. 터미널 4: rviz2
5. rviz2에서 Marker 추가: Add -> By topic -> /frame_trail_marker
6. Fixed Frame을 'world'로 설정

rviz2 설정 팁:
- Global Options -> Fixed Frame: "world"
- Marker 디스플레이 설정 확인
- TF 디스플레이도 추가하면 좌표계와 궁적을 함께 볼 수 있음

작성자: 학습용 예제
날짜: 2024
"""

# ROS 2 파이썬 클라이언트 라이브러리
import rclpy
from rclpy.node import Node  # ROS 2 노드 기본 클래스
# TF2 라이브러리 (좌표계 변환 조회용)
import tf2_ros
# 수학 연산 라이브러리 (현재는 사용하지 않지만 확장 가능)
import math

# 기하학 메시지 타입 (3D 점 좌표)
from geometry_msgs.msg import Point
# 시각화 메시지 타입 (rviz2에서 도형 표시용)
from visualization_msgs.msg import Marker

class FrameTrailPublisher(Node):
    """
    TF 궁적 시각화 Publisher 클래스
    
    이 클래스는 지정된 좌표계의 실시간 위치를 추적하여 그 이동 경로를
    rviz2에서 시각화할 수 있는 LINE_STRIP 마커를 생성하고 발행합니다.
    
    주요 기능:
    - 지정된 target_frame의 실시간 위치 조회
    - 위치 데이터를 내부 리스트에 누적 저장
    - 누적된 위치들을 연결하는 LINE_STRIP 마커 생성
    - rviz2에서 시각화 가능한 Marker 메시지 발행
    
    시각화 결과:
    - 원운동하는 child_frame의 경로가 빨간색 선으로 표시
    - 실시간으로 궁적이 연장되며 완전한 원 모양 형성
    - 3D 공간에서의 이동 경로 시각화
    
    설정 가능 옵션:
    - target_frame: 추적할 좌표계 이름
    - source_frame: 기준 좌표계 이름
    - 마커 색상, 크기, 수명 등
    """
    def __init__(self):
        """
        노드 초기화 함수
        
        TF 시스템, Marker Publisher, 궁적 데이터 저장소, 타이머를 설정합니다.
        사용자가 추적하고 싶은 좌표계를 지정할 수 있습니다.
        """
        # 부모 클래스(Node) 초기화 - 노드 이름을 'frame_trail_publisher'로 설정
        super().__init__('frame_trail_publisher')

        # === 추적 대상 좌표계 설정 ===
        # 어떤 프레임의 궁적(잔상)을 그리고 싶은지 지정
        self.target_frame = 'child_frame'  # 추적할 좌표계 (원운동하는 프레임)
        self.source_frame = 'world'        # 기준 좌표계 (고정된 기준점)
        
        # 다른 프레임을 추적하고 싶다면 여기서 변경 가능:
        # self.target_frame = 'moving_frame'  # moving_frame의 궁적을 그리고 싶을 때

        # === TF 조회 시스템 설정 ===
        # TF buffer: 과거 TF 정보를 일정 시간 동안 저장하는 메모리 버퍼
        self.tf_buffer = tf2_ros.Buffer()
        # TF listener: 네트워크에서 TF 정보를 실시간으로 수신하여 Buffer에 저장
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # === Marker Publisher 설정 ===
        # Marker 메시지를 'frame_trail_marker' 토픽으로 발행
        # rviz2에서 이 토픽을 구독하여 시각화
        # 큐 크기: 10 (마커 메시지는 비교적 크므로 적절한 버퍼링)
        self.marker_pub = self.create_publisher(Marker, 'frame_trail_marker', 10)

        # === 궁적 데이터 저장소 ===
        # 실시간으로 수집한 위치 데이터를 누적적으로 저장하는 리스트
        # 각 원소는 (x, y, z) 튜플 형태
        self.positions = []

        # === 타이머 설정 ===
        # 0.1초(100ms) 간격으로 timer_callback 함수를 호출 (10Hz 주파수)
        # 이 주기로 TF를 조회하고 Marker를 업데이트
        # 너무 빠르면 과도한 데이터, 너무 느리면 부드럽지 않은 궁적
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 노드 시작 로그 출력
        self.get_logger().info(f'궁적 시각화 노드가 시작되었습니다.')
        self.get_logger().info(f'추적 대상: {self.target_frame} (기준: {self.source_frame})')
        self.get_logger().info('rviz2에서 /frame_trail_marker 토픽을 추가하여 궁적을 확인하세요!')

    def timer_callback(self):
        """
        타이머 콜백 함수 - 주기적 TF 조회 및 마커 업데이트 (10Hz)
        
        이 함수는 0.1초마다 호출되어 다음 작업을 수행합니다:
        1. TF Buffer에서 source_frame -> target_frame 변환 조회
        2. 변환 정보에서 위치 좌표 (x,y,z) 추출
        3. 추출한 위치를 궁적 데이터 리스트에 추가
        4. 모든 누적된 위치들을 연결하는 LINE_STRIP 마커 생성
        5. 마커 메시지를 rviz2로 발행
        """
        # === TF 변환 조회 ===
        # 현재 시각 기준으로 transform lookup 수행
        # timeout 인자를 추가하면 조금 과거 데이터도 허용 가능
        try:
            # lookup_transform()을 사용하여 target_frame의 source_frame 기준 위치 조회
            trans = self.tf_buffer.lookup_transform(
                self.source_frame,  # 기준 좌표계 (world)
                self.target_frame,  # 추적할 좌표계 (child_frame)
                rclpy.time.Time()   # 시간 (빈 Time()은 "가장 최근" 의미)
            )
        except Exception as e:
            # TF 조회 실패 시 경고 메시지 출력 후 함수 종료
            # 일반적인 실패 원인: TF 발행 노드가 아직 시작되지 않음
            self.get_logger().warn(f"{self.source_frame}에서 {self.target_frame}으로의 변환 실패: {e}")
            return  # 이번 주기는 건너뛰고 다음 호출을 기다림

        # === 위치 데이터 추출 및 누적 ===
        # TransformStamped 메시지에서 translation (위치) 정보 추출
        x = trans.transform.translation.x  # X축 좌표 (미터 단위)
        y = trans.transform.translation.y  # Y축 좌표 (미터 단위)
        z = trans.transform.translation.z  # Z축 좌표 (미터 단위)
        
        # 새로 얻은 위치를 궁적 데이터 리스트에 추가
        # 각 위치는 (x, y, z) 튜플로 저장
        self.positions.append((x, y, z))

        # === 메모리 관리 (선택적) ===
        # 만약 궁적이 너무 길어져서 메모리를 많이 사용하게 되면
        # 오래된 데이터를 제거하여 메모리 사용량을 제한할 수 있음
        # 예: 1000개를 초과하면 가장 오래된 데이터를 삭제
        # 현재는 주석 처리되어 있음 (무제한 누적)
        # if len(self.positions) > 1000:
        #     self.positions.pop(0)  # 리스트의 첫 번째 원소 (가장 오래된 데이터) 삭제

        # === Marker 메시지 생성 ===
        # rviz2에서 시각화할 Marker 메시지 객체 생성
        marker = Marker()
        
        # 헤더 정보 설정
        marker.header.frame_id = self.source_frame  # 마커가 표시될 기준 좌표계
        marker.header.stamp = self.get_clock().now().to_msg()  # 현재 시간 스탬프

        # 마커 식별 정보
        marker.ns = "frame_trail"  # 네임스페이스 (다른 마커와 구별용)
        marker.id = 0              # 마커 ID (같은 ns 내에서 고유해야 함)
        
        # 마커 타입 및 동작 설정
        marker.type = Marker.LINE_STRIP  # 선분 체인 타입 (점들을 순서대로 연결)
        marker.action = Marker.ADD       # 마커 추가 동작 (DELETE로 삭제 가능)

        # === LINE_STRIP을 구성할 점들 설정 ===
        # 누적된 모든 위치 데이터를 Point 연속으로 변환
        marker.points = []
        for pos in self.positions:
            # 각 (x, y, z) 튜플을 geometry_msgs/Point 메시지로 변환
            p = Point()
            p.x = pos[0]  # X 좌표
            p.y = pos[1]  # Y 좌표
            p.z = pos[2]  # Z 좌표
            marker.points.append(p)  # 마커의 점 리스트에 추가

        # === 마커 시각적 속성 설정 ===
        # 선 굵기 (너비)
        marker.scale.x = 0.02  # 2cm 정도 두께 (LINE_STRIP에서는 x만 사용)
        # scale.y와 scale.z는 LINE_STRIP에서 사용되지 않음

        # 색상 설정 (RGBA 값, 0.0~1.0 범위)
        marker.color.r = 1.0  # 빨간색 성분 (최대)
        marker.color.g = 0.2  # 녹색 성분 (약간)
        marker.color.b = 0.2  # 파란색 성분 (약간)
        marker.color.a = 1.0  # 알파 값 (투명도, 1.0=불투명, 0.0=완전투명)
        # 결과: 댄은 빨간색 선

        # 마커 수명 설정
        marker.lifetime.sec = 0  # 0이면 영구 표시, 양수면 해당 초 후 자동 삭제
        # marker.lifetime.nanosec = 0  # 나노초 단위 수명 (선택적)

        # === 마커 발행 ===
        # 완성된 마커 메시지를 rviz2로 발행
        # rviz2에서 /frame_trail_marker 토픽을 구독하면 실시간 궁적 확인 가능
        self.marker_pub.publish(marker)
        
        # 디버깅용 로그 (너무 자주 출력되지 않도록 조건부)
        # 매 50번째 호출마다 로그 출력 (5초마다)
        if len(self.positions) % 50 == 0:
            self.get_logger().info(f'궁적 업데이트: 총 {len(self.positions)}개 점, 최신 위치({x:.2f}, {y:.2f}, {z:.2f})')

def main(args=None):
    """
    메인 함수 - 프로그램 진입점
    
    궁적 시각화 노드를 초기화하고 실행합니다.
    다른 TF 발행 노드들이 먼저 실행되어야 정상 동작합니다.
    
    Args:
        args: 명령행 인수 (기본값: None)
    """
    # ROS 2 시스템 초기화
    rclpy.init(args=args)
    
    # 궁적 시각화 노드 객체 생성
    node = FrameTrailPublisher()
    
    try:
        # 노드 실행 시작 (무한 루프)
        # TF 정보를 실시간으로 모니터링하며 궁적을 그립니다
        print("=== 궁적 시각화 노드 실행 중 ===")
        print("전제 조건:")
        print("1. my_tf_1 노드가 실행 중이어야 함")
        print("2. child_frame 노드가 실행 중이어야 함")
        print("")
        print("시각화 방법:")
        print("1. rviz2 실행")
        print("2. Add -> By topic -> /frame_trail_marker 선택")
        print("3. Fixed Frame을 'world'로 설정")
        print("종료하려면 Ctrl+C를 누르세요.")
        print("==========================================")
        
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
        print("궁적 시각화 노드가 정상적으로 종료되었습니다.")

# 스크립트가 직접 실행될 때만 main() 함수 호출
# 다른 모듈에서 import될 때는 실행되지 않음
if __name__ == '__main__':
    main()