# simple_navigator.py (PID 제어 및 상태 머신 적용 버전)
"""
🤖 ROS2 Simple Navigator with PID Control & State Machine

이 모듈은 로봇이 목표 지점까지 자율 주행할 수 있도록 하는 간단한 네비게이션 시스템입니다.

주요 기능:
- PID 제어를 통한 정밀한 속도 및 방향 제어
- 상태 머신을 통한 체계적인 주행 단계 관리
- TF(Transform) 시스템을 활용한 실시간 위치 추적
- ROS2 토픽 통신을 통한 목표점 수신 및 속도 명령 송신

상태 머신 흐름:
idle → rotate_to_goal → move_to_goal → rotate_to_final → idle
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist  # 위치/자세 및 속도 메시지
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener  # 좌표 변환 리스너
import tf_transformations  # 쿼터니언 ↔ 오일러각 변환
import math

class PID:
    """
    📊 PID (Proportional-Integral-Derivative) 제어기 클래스
    
    PID 제어는 목표값과 현재값의 오차를 이용해 제어 신호를 생성하는 방법입니다.
    - P (비례): 현재 오차에 비례한 제어 → 빠른 응답, 정상상태 오차 존재
    - I (적분): 오차의 누적값에 비례 → 정상상태 오차 제거, 오버슈트 위험
    - D (미분): 오차의 변화율에 비례 → 안정성 향상, 노이즈에 민감
    """
    def __init__(self, P=0.0, I=0.0, D=0.0):
        # PID 게인 상수들 (튜닝 파라미터)
        self.P = P  # 비례 게인: 클수록 빠른 응답, 너무 크면 진동
        self.I = I  # 적분 게인: 정상상태 오차 제거, 너무 크면 불안정
        self.D = D  # 미분 게인: 안정성 증가, 노이즈 감쇠
        
        # 제어 계산을 위한 내부 상태 변수들
        self.previous_error = 0.0  # 이전 시점의 오차 (미분 계산용)
        self.integral = 0.0        # 오차의 누적합 (적분 계산용)

    def update(self, error):
        """
        🎯 PID 제어 신호 계산
        
        Args:
            error: 목표값 - 현재값 (예: 목표각도 - 현재각도)
            
        Returns:
            제어 신호 (예: 각속도 명령)
        """
        # I항: 오차를 시간에 대해 적분 (누적 오차)
        self.integral += error
        
        # D항: 오차의 시간에 대한 미분 (오차 변화율)
        derivative = error - self.previous_error
        self.previous_error = error
        
        # PID 제어 법칙: u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
        return self.P * error + self.I * self.integral + self.D * derivative

    def reset(self):
        """
        🔄 PID 제어기 초기화
        
        새로운 목표가 설정될 때 호출하여 이전 제어 이력을 초기화합니다.
        이를 통해 이전 목표의 누적 오차가 새 목표에 영향을 주는 것을 방지합니다.
        """
        self.previous_error = 0.0  # 이전 오차 초기화
        self.integral = 0.0        # 누적 오차 초기화

def normalize_angle(angle):
    """
    📐 각도 정규화 함수
    
    각도를 -π ~ π 범위로 정규화합니다.
    
    ❓ 왜 필요한가?
    로봇의 방향각은 0~2π 또는 -π~π로 표현되는데, 계산 과정에서
    범위를 벗어날 수 있습니다. 예를 들어, 350°에서 10°로 회전할 때
    단순 계산하면 -340° 회전으로 인식되어 반대 방향으로 돌 수 있습니다.
    
    🔧 동작 원리:
    atan2(sin(θ), cos(θ))를 사용하면 자동으로 -π~π 범위로 정규화됩니다.
    
    Args:
        angle: 정규화할 각도 (라디안)
        
    Returns:
        -π ~ π 범위로 정규화된 각도
    """
    return math.atan2(math.sin(angle), math.cos(angle))

class SimpleNavigator(Node):
    """
    🚀 Simple Navigator 클래스
    
    ROS2 Node를 상속받아 로봇 자율주행 기능을 구현합니다.
    상태 머신과 PID 제어를 결합하여 목표점까지 안전하고 정확하게 이동합니다.
    """
    
    def __init__(self):
        # ROS2 Node 초기화 ('simple_navigator_pid'라는 이름으로 노드 생성)
        super().__init__('simple_navigator_pid')

        # 🎯 제어 파라미터 설정
        # 허용 오차가 작을수록 정밀하지만 도달하기 어려워집니다
        self.angle_tolerance = 0.05    # 각도 허용 오차 (≈ 2.9도)
        self.distance_tolerance = 0.15  # 거리 허용 오차 (15cm)
        
        # 🎛️ PID 제어기 초기화 (게인값은 실험을 통해 튜닝)
        # 각속도 제어용: 로봇의 회전을 제어
        self.angular_pid = PID(P=1.5, I=0.0, D=0.2)  # P값이 큰 이유: 빠른 회전 응답 필요
        # 선속도 제어용: 로봇의 전진을 제어  
        self.linear_pid = PID(P=0.5, I=0.0, D=0.1)   # P값이 작은 이유: 부드러운 가속 필요
        
        # 🗺️ TF (Transform) 시스템 설정
        # TF는 ROS에서 좌표계 간의 변환을 관리하는 시스템입니다
        self.tf_buffer = Buffer()  # 변환 정보를 저장하는 버퍼
        self.tf_listener = TransformListener(self.tf_buffer, self)  # 변환 정보 수신기
        # 'map' → 'base_link' 변환을 통해 로봇의 글로벌 위치를 얻습니다

        # 📡 ROS2 통신 설정
        # Publisher: 로봇에게 속도 명령을 전송
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # Subscriber: 외부에서 목표 위치를 수신
        self.subscription = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10)
        
        # ⏰ 제어 루프 타이머 (10Hz = 0.1초마다 실행)
        # 주기적 실행을 통해 실시간 제어를 구현합니다
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # 🎭 상태 머신 변수들
        self.goal_pose = None    # 현재 목표 위치 (None이면 목표 없음)
        self.state = "idle"      # 현재 상태 (idle/rotate_to_goal/move_to_goal/rotate_to_final)
        
        self.get_logger().info('Simple PID Navigator has been started.')

    def goal_callback(self, msg):
        """
        🎯 새로운 목표점 수신 콜백 함수
        
        외부에서 goal_pose 토픽으로 목표 위치가 전송되면 호출됩니다.
        상태 머신을 초기화하고 새로운 네비게이션을 시작합니다.
        
        Args:
            msg (PoseStamped): 목표 위치와 방향 정보
        """
        self.goal_pose = msg              # 새 목표점 저장
        self.state = "rotate_to_goal"     # 첫 번째 단계: 목표 방향으로 회전
        
        # 🔄 PID 제어기 초기화 (이전 목표의 누적 오차 제거)
        self.angular_pid.reset()
        self.linear_pid.reset()
        
        self.get_logger().info(f'🎯 New goal received. State: {self.state}')

    def control_loop(self):
        """
        🔄 메인 제어 루프 (10Hz로 실행)
        
        상태 머신의 현재 상태에 따라 적절한 제어 로직을 실행합니다.
        이 함수가 로봇 자율주행의 핵심 역할을 담당합니다.
        """
        # 🛑 목표가 없거나 idle 상태면 로봇 정지
        if self.goal_pose is None or self.state == "idle":
            self.publisher_.publish(Twist())  # 모든 속도를 0으로 설정
            return
        
        try:
            # 🗺️ TF를 통해 로봇의 현재 위치 획득
            # 'map' → 'base_link' 변환: 글로벌 좌표계에서 로봇의 위치
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            
            # 📍 위치 정보 추출
            current_x = trans.transform.translation.x  # X 좌표 (전후)
            current_y = trans.transform.translation.y  # Y 좌표 (좌우)
            
            # 🧭 방향 정보 추출 (쿼터니언 → 오일러각 변환)
            q = trans.transform.rotation
            _, _, current_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            # current_yaw: 로봇이 바라보는 방향 (Z축 회전각)
            
        except Exception as e:
            # TF 변환 실패 시 (예: 센서 데이터 부족)
            self.get_logger().warn(f'❌ Could not get robot pose: {e}')
            return

        # 🎭 상태 머신: 현재 상태에 따른 제어 로직 실행
        twist_msg = Twist()
        
        if self.state == "rotate_to_goal":
            # 1단계: 목표점을 향해 회전
            twist_msg = self.handle_rotate_to_goal(current_x, current_y, current_yaw)
        elif self.state == "move_to_goal":
            # 2단계: 목표점까지 직진 (동시에 방향 보정)
            twist_msg = self.handle_move_to_goal(current_x, current_y, current_yaw)
        elif self.state == "rotate_to_final":
            # 3단계: 최종 목표 방향으로 회전
            twist_msg = self.handle_rotate_to_final(current_yaw)
        
        # 🚀 계산된 속도 명령을 로봇에 전송
        self.publisher_.publish(twist_msg)

    def handle_rotate_to_goal(self, current_x, current_y, current_yaw):
        """
        🔄 1단계: 목표 지점을 향해 회전
        
        로봇이 목표점을 바라보도록 제자리에서 회전합니다.
        정확한 방향 정렬 후 다음 단계로 진행합니다.
        """
        twist_msg = Twist()
        
        # 📐 목표점까지의 방향각 계산
        desired_heading = math.atan2(self.goal_pose.pose.position.y - current_y,
                                   self.goal_pose.pose.position.x - current_x)
        
        # 🎯 각도 오차 계산 (목표방향 - 현재방향)
        error_angle = normalize_angle(desired_heading - current_yaw)

        if abs(error_angle) > self.angle_tolerance:
            # 🔄 아직 정렬되지 않음 → PID로 각속도 계산
            twist_msg.angular.z = self.angular_pid.update(error_angle)
        else:
            # ✅ 방향 정렬 완료 → 다음 단계로 전환
            twist_msg.angular.z = 0.0
            self.state = "move_to_goal"
            self.get_logger().info(f'✅ Heading aligned. State: {self.state}')
            
        return twist_msg

    def handle_move_to_goal(self, current_x, current_y, current_yaw):
        """
        🚀 2단계: 목표 지점까지 직진 이동
        
        목표점까지 전진하면서 동시에 방향을 보정합니다.
        이를 통해 곡선 경로로도 목표점에 도달할 수 있습니다.
        """
        twist_msg = Twist()
        
        # 📏 목표점까지의 거리 계산
        dx = self.goal_pose.pose.position.x - current_x
        dy = self.goal_pose.pose.position.y - current_y
        distance_error = math.sqrt(dx**2 + dy**2)  # 유클리드 거리

        if distance_error > self.distance_tolerance:
            # 🚀 아직 목표에 도달하지 않음 → 전진 및 방향 보정
            
            # 거리 오차를 이용한 선속도 제어 (멀수록 빠르게)
            twist_msg.linear.x = self.linear_pid.update(distance_error)
            
            # 🧭 주행 중 방향 보정 (목표점을 계속 바라보며 이동)
            desired_heading = math.atan2(dy, dx)
            error_angle = normalize_angle(desired_heading - current_yaw)
            twist_msg.angular.z = self.angular_pid.update(error_angle)
            
        else:
            # ✅ 목표 위치 도달 → 정지 후 다음 단계로
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.state = "rotate_to_final"
            self.get_logger().info(f'🎯 Position reached. State: {self.state}')
            
        return twist_msg

    def handle_rotate_to_final(self, current_yaw):
        """
        🎯 3단계: 최종 목표 방향으로 회전
        
        목표 위치에 도달한 후, 목표에서 지정한 최종 방향으로 로봇을 회전시킵니다.
        이 단계 완료 후 네비게이션이 완전히 종료됩니다.
        """
        twist_msg = Twist()
        
        # 🧭 목표에서 지정한 최종 방향각 추출
        q = self.goal_pose.pose.orientation
        _, _, final_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # 📐 최종 방향 오차 계산
        final_error_angle = normalize_angle(final_yaw - current_yaw)

        if abs(final_error_angle) > self.angle_tolerance:
            # 🔄 아직 최종 방향에 도달하지 않음
            twist_msg.angular.z = self.angular_pid.update(final_error_angle)
        else:
            # 🎉 네비게이션 완료! idle 상태로 복귀
            twist_msg.angular.z = 0.0
            self.state = "idle"     # 상태 초기화
            self.goal_pose = None   # 목표 정보 삭제
            self.get_logger().info(f'🎉 Final orientation reached. Goal achieved! State: {self.state}')
            
        return twist_msg

def main(args=None):
    """
    🚀 메인 함수: ROS2 노드 실행 및 관리
    
    노드 생성, 실행, 종료를 관리합니다.
    """
    # ROS2 시스템 초기화
    rclpy.init(args=args)
    
    # SimpleNavigator 노드 생성
    node = SimpleNavigator()
    
    try:
        # 노드 실행 (무한 루프, 콜백 함수들이 호출됨)
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C로 종료 시
        node.get_logger().info('🛑 Keyboard interrupt, shutting down.')
    finally:
        # 정리 작업
        node.destroy_node()  # 노드 소멸
        rclpy.shutdown()     # ROS2 시스템 종료

if __name__ == '__main__':
    # 스크립트가 직접 실행될 때만 main() 호출
    main()