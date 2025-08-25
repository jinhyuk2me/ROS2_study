#!/usr/bin/env python3
"""
고급 터틀 목표 제어기 (Advanced Turtle Goal Controller)

이 파일은 상태 기반 제어(State Machine)와 듀얼 PID를 활용하여 터틀을 정밀하게 
목표 위치와 방향으로 이동시키는 고급 제어기입니다.

주요 특징:
1. 상태 기반 제어 (Finite State Machine)
2. 듀얼 PID 제어 (각속도 + 선속도 독립 제어)
3. 동적 파라미터 재구성 (실시간 PID 튜닝)
4. 3단계 정밀 제어 프로세스
5. 실시간 상태 모니터링

제어 흐름 (State Machine):
┌─────────────────────────────────────────────────────────────┐
│                    터틀 제어 상태 다이어그램                  │
├─────────────────────────────────────────────────────────────┤
│  [idle] → 목표 수신 → [rotate_to_goal] → 방향 정렬 완료     │
│                           ↓                                 │
│  [goal_reached] ← 최종 방향 완료 ← [rotate_to_final]        │
│           ↑                              ↑                  │
│           └── 위치 도달 완료 ← [move_to_goal]               │
└─────────────────────────────────────────────────────────────┘

각 상태별 제어 목적:
- idle: 대기 상태 (목표 없음)
- rotate_to_goal: 목표 방향으로 회전 (이동 전 정렬)
- move_to_goal: 목표 위치로 이동 (방향 보정 포함)
- rotate_to_final: 최종 방향으로 회전 (정밀 자세 제어)
- goal_reached: 목표 달성 (정지 상태)

학습 목표:
- 상태 기반 제어 시스템 설계 및 구현
- 복수 PID 제어기의 독립적 운용
- ROS2 파라미터 동적 재구성
- 로봇 제어에서의 정밀도와 안정성 확보
- 실시간 상태 모니터링 및 디버깅

pose_dual_controller.py와의 비교:
- pose_dual_controller: 단순 듀얼 PID (위치 + 방향 동시 제어)
- move_turtle: 상태 기반 순차 제어 (단계별 정밀 제어)

작성자: [작성자명]
날짜: [작성일]
"""

# =============================================================================
# 라이브러리 import 섹션
# =============================================================================

# ROS2 관련 라이브러리
import rclpy                                    # ROS2 Python 클라이언트 라이브러리
from rclpy.node import Node                     # ROS2 노드 기본 클래스

# ROS2 메시지 타입들
from turtlesim.msg import Pose                  # 터틀 위치/방향 정보 (x, y, theta)
from geometry_msgs.msg import Twist             # 속도 명령 (linear.x, angular.z)
from std_msgs.msg import Float64, String        # 기본 데이터 타입 (에러값, 상태 문자열)
from rcl_interfaces.msg import SetParametersResult  # 파라미터 설정 결과

# 수학 및 제어 라이브러리
import math                                     # 수학 함수 (삼각함수, 각도 계산)
from controller_tutorials.control_apps import PID  # 커스텀 PID 클래스

# =============================================================================
# 유틸리티 함수 섹션
# =============================================================================

def normalize_angle(angle):
    """
    각도 정규화 함수 (핵심 수학 유틸리티)
    
    임의의 각도를 [-π, π] 범위로 정규화합니다.
    이는 각도 오차 계산 시 최단 회전 경로를 보장하기 위해 필수적입니다.
    
    수학적 원리:
    - atan2(sin(θ), cos(θ)) = θ (단, -π ≤ θ ≤ π)
    - 이 공식은 각도를 단위원 상의 점으로 변환 후 다시 각도로 변환
    - 결과적으로 2π의 배수만큼 차이나는 각도들을 동일하게 처리
    
    예시:
    - normalize_angle(3π) = π
    - normalize_angle(-3π) = -π
    - normalize_angle(π/2) = π/2
    
    Args:
        angle (float): 정규화할 각도 (라디안)
        
    Returns:
        float: 정규화된 각도 (-π ≤ angle ≤ π)
    """
    return math.atan2(math.sin(angle), math.cos(angle))

# =============================================================================
# 메인 제어기 클래스 정의
# =============================================================================

class TurtleGoalController(Node):
    """
    고급 터틀 목표 제어기 클래스
    
    이 클래스는 상태 기반 제어(Finite State Machine)와 듀얼 PID를 활용하여
    터틀을 정밀하게 목표 위치와 방향으로 이동시키는 고급 제어기입니다.
    
    주요 구성요소:
    1. 상태 관리: 5가지 상태 (idle, rotate_to_goal, move_to_goal, rotate_to_final, goal_reached)
    2. 듀얼 PID: 각속도 제어용 PID + 선속도 제어용 PID
    3. 동적 파라미터: 실시간 PID 게인 및 허용 오차 조정
    4. 토픽 통신: 위치 구독, 목표 구독, 속도/에러/상태 발행
    
    제어 철학:
    - 정밀도 우선: 각 단계를 순차적으로 완료하여 높은 정확도 달성
    - 안정성 확보: 급격한 방향 변화 없이 부드러운 이동
    - 모니터링 지원: 실시간 상태 및 에러 정보 제공
    """
    
    def __init__(self):
        """
        터틀 제어기 노드 초기화
        
        초기화 과정:
        1. ROS2 파라미터 선언 및 기본값 설정
        2. 듀얼 PID 제어기 생성 및 구성
        3. 상태 변수 및 목표 변수 초기화
        4. ROS2 토픽 구독자/발행자 설정
        5. 동적 파라미터 콜백 등록
        """
        # 부모 클래스(Node) 초기화
        super().__init__('turtle_goal_controller')
        
        # =============================================================================
        # 1. ROS2 파라미터 선언 (동적 재구성 가능)
        # =============================================================================
        
        # 허용 오차 파라미터 (제어 정밀도 결정)
        self.declare_parameter('angle_tolerance', 0.01)      # 각도 허용 오차 (라디안)
        self.declare_parameter('distance_tolerance', 0.01)   # 거리 허용 오차 (미터)
        
        # 각속도 PID 제어기 파라미터
        self.declare_parameter('angular_P', 1.0)             # 비례 게인 (즉시 반응)
        self.declare_parameter('angular_I', 0.0)             # 적분 게인 (정상상태 오차 제거)
        self.declare_parameter('angular_D', 0.0)             # 미분 게인 (진동 억제)
        self.declare_parameter('angular_max_state', 2.0)     # 최대 각속도 (rad/s)
        self.declare_parameter('angular_min_state', -2.0)    # 최소 각속도 (rad/s)
        
        # 선속도 PID 제어기 파라미터
        self.declare_parameter('linear_P', 1.0)              # 비례 게인
        self.declare_parameter('linear_I', 0.0)              # 적분 게인
        self.declare_parameter('linear_D', 0.0)              # 미분 게인
        self.declare_parameter('linear_max_state', 2.0)      # 최대 선속도 (m/s)
        self.declare_parameter('linear_min_state', -2.0)     # 최소 선속도 (m/s)
        
        # 허용 오차 파라미터 값 읽기
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        
        # =============================================================================
        # 2. 듀얼 PID 제어기 생성 및 구성
        # =============================================================================
        
        # 각속도 PID 제어기 초기화 (방향 제어용)
        angular_P = self.get_parameter('angular_P').value
        angular_I = self.get_parameter('angular_I').value
        angular_D = self.get_parameter('angular_D').value
        angular_max_state = self.get_parameter('angular_max_state').value
        angular_min_state = self.get_parameter('angular_min_state').value
        
        self.angular_pid = PID()                           # 각속도 PID 인스턴스 생성
        self.angular_pid.P = angular_P                     # 비례 게인 설정
        self.angular_pid.I = angular_I                     # 적분 게인 설정
        self.angular_pid.D = angular_D                     # 미분 게인 설정
        self.angular_pid.max_state = angular_max_state     # 최대 출력 제한
        self.angular_pid.min_state = angular_min_state     # 최소 출력 제한
        
        # 선속도 PID 제어기 초기화 (거리 제어용)
        linear_P = self.get_parameter('linear_P').value
        linear_I = self.get_parameter('linear_I').value
        linear_D = self.get_parameter('linear_D').value
        linear_max_state = self.get_parameter('linear_max_state').value
        linear_min_state = self.get_parameter('linear_min_state').value
        
        self.linear_pid = PID()                            # 선속도 PID 인스턴스 생성
        self.linear_pid.P = linear_P                       # 비례 게인 설정
        self.linear_pid.I = linear_I                       # 적분 게인 설정
        self.linear_pid.D = linear_D                       # 미분 게인 설정
        self.linear_pid.max_state = linear_max_state       # 최대 출력 제한
        self.linear_pid.min_state = linear_min_state       # 최소 출력 제한

        # =============================================================================
        # 3. 상태 변수 및 목표 변수 초기화
        # =============================================================================
        
        # 상태 기반 제어를 위한 현재 상태 (State Machine의 핵심)
        # 가능한 상태: "idle", "rotate_to_goal", "move_to_goal", "rotate_to_final", "goal_reached"
        self.state = "idle"
        
        # 목표 위치 및 방향 정보 저장 변수
        # goal_pose 토픽에서 수신한 목표 정보를 저장 (초기값 None)
        self.goal_pose = None
        
        # =============================================================================
        # 4. ROS2 토픽 구독자 및 발행자 설정
        # =============================================================================
        
        # 구독자 (Subscriber) 설정 - 데이터 수신용
        self.pose_subscriber = self.create_subscription(
            Pose,                    # 메시지 타입: 터틀의 현재 위치/방향
            'turtle1/pose',          # 토픽 이름: turtlesim이 발행하는 터틀 상태
            self.pose_callback,      # 콜백 함수: 새 위치 데이터 처리
            10                       # 큐 크기: 최대 10개 메시지 버퍼링
        )
        
        self.goal_pose_subscriber = self.create_subscription(
            Pose,                    # 메시지 타입: 목표 위치/방향
            'goal_pose',             # 토픽 이름: 목표 설정 토픽
            self.goal_pose_callback, # 콜백 함수: 새 목표 처리
            10                       # 큐 크기
        )
        
        # 발행자 (Publisher) 설정 - 명령 및 상태 정보 전송용
        self.cmd_vel_publisher = self.create_publisher(
            Twist,                   # 메시지 타입: 속도 명령 (linear.x, angular.z)
            'turtle1/cmd_vel',       # 토픽 이름: 터틀 제어 명령 토픽
            10                       # 큐 크기
        )
        
        self.error_publisher = self.create_publisher(
            Float64,                 # 메시지 타입: 실수 (현재 제어 오차)
            'error',                 # 토픽 이름: 오차 모니터링용
            10                       # 큐 크기
        )
        
        self.state_publisher = self.create_publisher(
            String,                  # 메시지 타입: 문자열 (현재 상태)
            'turtle_state',          # 토픽 이름: 상태 모니터링용
            10                       # 큐 크기
        )
        
        # =============================================================================
        # 5. 동적 파라미터 재구성 설정
        # =============================================================================
        
        # 실시간 파라미터 변경을 위한 콜백 함수 등록
        # 이를 통해 PID 게인이나 허용 오차를 런타임에 조정 가능
        self.add_on_set_parameters_callback(self.parameter_callback)
    
    # =============================================================================
    # 유틸리티 및 콜백 메서드들
    # =============================================================================
    
    def publish_state(self):
        """
        현재 상태를 ROS2 토픽으로 발행
        
        이 메서드는 상태 변화 시마다 호출되어 외부에서 현재 제어 상태를 
        모니터링할 수 있도록 합니다. GUI나 다른 노드에서 활용 가능합니다.
        """
        msg = String()
        msg.data = self.state                    # 현재 상태 문자열
        self.state_publisher.publish(msg)       # turtle_state 토픽으로 발행
    
    def parameter_callback(self, params):
        """
        동적 파라미터 재구성 콜백 함수
        
        ROS2의 파라미터 서버를 통해 실시간으로 제어 파라미터를 변경할 수 있게 해주는
        핵심 기능입니다. 이를 통해 터틀이 동작 중에도 PID 게인을 튜닝할 수 있습니다.
        
        지원하는 파라미터:
        - 허용 오차: angle_tolerance, distance_tolerance
        - 각속도 PID: angular_P, angular_I, angular_D, angular_max/min_state
        - 선속도 PID: linear_P, linear_I, linear_D, linear_max/min_state
        
        사용법 예시:
        ros2 param set /turtle_goal_controller angular_P 2.0
        ros2 param set /turtle_goal_controller angle_tolerance 0.05
        
        Args:
            params: 변경된 파라미터 리스트
            
        Returns:
            SetParametersResult: 파라미터 설정 성공 여부
        """
        for param in params:
            # 허용 오차 파라미터 업데이트
            if param.name == 'angle_tolerance':
                self.angle_tolerance = param.value
                self.get_logger().info(f"Updated angle_tolerance: {param.value}")
            elif param.name == 'distance_tolerance':
                self.distance_tolerance = param.value
                self.get_logger().info(f"Updated distance_tolerance: {param.value}")
            
            # 각속도 PID 파라미터 업데이트
            elif param.name == 'angular_P':
                self.angular_pid.P = param.value
                self.get_logger().info(f"Updated angular_PID P: {param.value}")
            elif param.name == 'angular_I':
                self.angular_pid.I = param.value
                self.get_logger().info(f"Updated angular_PID I: {param.value}")
            elif param.name == 'angular_D':
                self.angular_pid.D = param.value
                self.get_logger().info(f"Updated angular_PID D: {param.value}")
            elif param.name == 'angular_max_state':
                self.angular_pid.max_state = param.value
                self.get_logger().info(f"Updated angular_PID max_state: {param.value}")
            elif param.name == 'angular_min_state':
                self.angular_pid.min_state = param.value
                self.get_logger().info(f"Updated angular_PID min_state: {param.value}")
                
            # 선속도 PID 파라미터 업데이트
            elif param.name == 'linear_P':
                self.linear_pid.P = param.value
                self.get_logger().info(f"Updated linear_PID P: {param.value}")
            elif param.name == 'linear_I':
                self.linear_pid.I = param.value
                self.get_logger().info(f"Updated linear_PID I: {param.value}")
            elif param.name == 'linear_D':
                self.linear_pid.D = param.value
                self.get_logger().info(f"Updated linear_PID D: {param.value}")
            elif param.name == 'linear_max_state':
                self.linear_pid.max_state = param.value
                self.get_logger().info(f"Updated linear_PID max_state: {param.value}")
            elif param.name == 'linear_min_state':
                self.linear_pid.min_state = param.value
                self.get_logger().info(f"Updated linear_PID min_state: {param.value}")
                
        return SetParametersResult(successful=True)
    
    def goal_pose_callback(self, msg):
        """
        목표 위치 수신 콜백 함수
        
        새로운 목표가 설정되면 호출되는 함수로, 상태 기반 제어의 시작점입니다.
        목표를 저장하고 상태를 'rotate_to_goal'로 전환하여 제어 시퀀스를 시작합니다.
        
        제어 시퀀스 시작:
        idle → rotate_to_goal (목표 방향으로 회전 시작)
        
        Args:
            msg (Pose): 목표 위치 및 방향 정보
                       - msg.x, msg.y: 목표 위치 좌표
                       - msg.theta: 목표 방향각
        """
        # 새로운 목표 저장
        self.goal_pose = msg
        
        # 목표가 설정되면 즉시 제어 시퀀스 시작
        if self.goal_pose is not None:
            self.state = "rotate_to_goal"          # 첫 번째 단계: 목표 방향으로 회전
            self.publish_state()                   # 상태 변화 알림
            
        self.get_logger().info(
            f"Received new goal pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}"
        )
    
    def pose_callback(self, msg):
        """
        터틀 위치 수신 콜백 함수
        
        turtlesim에서 터틀의 현재 위치가 업데이트될 때마다 호출됩니다.
        목표가 설정된 상태에서만 제어 로직을 실행합니다.
        
        Args:
            msg (Pose): 터틀의 현재 위치 및 방향 정보
        """
        # 목표가 설정되지 않은 경우 제어 비활성
        if self.goal_pose is None:
            return
            
        # 현재 위치 정보로 제어 로직 실행
        self.control(msg)
    
    def control(self, current_pose):
        """
        상태 기반 메인 제어 함수 (State Machine Controller)
        
        이 함수는 현재 상태에 따라 적절한 제어 로직을 선택하여 실행합니다.
        각 상태별로 특화된 핸들러 함수를 호출하여 단계별 정밀 제어를 구현합니다.
        
        상태별 제어 전략:
        - rotate_to_goal: 목표 방향으로 회전 (각속도 제어만)
        - move_to_goal: 목표 위치로 이동 (선속도 + 방향 보정)
        - rotate_to_final: 최종 방향으로 회전 (각속도 제어만)
        - goal_reached: 정지 유지
        
        Args:
            current_pose (Pose): 터틀의 현재 위치 및 방향
        """
        # 기본 속도 명령 객체 생성
        twist_msg = Twist()
        
        # 현재 상태에 따른 제어 로직 분기
        if self.state == "rotate_to_goal":
            # 1단계: 목표 방향으로 회전
            twist_msg = self.handle_rotate_to_goal(current_pose)
        elif self.state == "move_to_goal":
            # 2단계: 목표 위치로 이동 (방향 보정 포함)
            twist_msg = self.handle_move_to_goal(current_pose)
        elif self.state == "rotate_to_final":
            # 3단계: 최종 방향으로 회전
            twist_msg = self.handle_rotate_to_final(current_pose)
        elif self.state == "goal_reached":
            # 4단계: 목표 달성 후 정지
            twist_msg = self.handle_goal_reached()
        
        # 계산된 속도 명령 터틀에 전송
        self.cmd_vel_publisher.publish(twist_msg)
    
    # =============================================================================
    # 상태별 제어 핸들러 함수들 (State-specific Control Handlers)
    # =============================================================================
    
    def handle_rotate_to_goal(self, current_pose):
        """
        1단계: 목표 방향으로 회전 제어 (Rotate to Goal Direction)
        
        터틀을 목표 위치 방향으로 회전시키는 단계입니다. 이동하기 전에 방향을 
        먼저 정렬함으로써 더 직선적이고 효율적인 경로를 확보합니다.
        
        제어 전략:
        - 각속도 제어만 사용 (선속도 = 0)
        - 목표 방향 각도 계산: atan2(Δy, Δx)
        - 각도 오차를 PID로 보정
        - 허용 오차 내 도달 시 다음 단계로 전환
        
        수학적 배경:
        desired_heading = atan2(goal_y - current_y, goal_x - current_x)
        error_angle = normalize_angle(desired_heading - current_theta)
        
        Args:
            current_pose (Pose): 터틀의 현재 위치 및 방향
            
        Returns:
            Twist: 속도 명령 (angular.z만 사용, linear.x = 0)
        """
        twist_msg = Twist()
        error_msg = Float64()
        
        # 목표 방향 계산 (현재 위치에서 목표 위치로의 방향각)
        desired_heading = math.atan2(self.goal_pose.y - current_pose.y,
                                     self.goal_pose.x - current_pose.x)
        
        # 각도 오차 계산 (정규화로 최단 회전 경로 보장)
        error_angle = normalize_angle(desired_heading - current_pose.theta)
        
        # 오차 정보 발행 (모니터링 및 분석용)
        error_msg.data = error_angle
        self.error_publisher.publish(error_msg)
        self.get_logger().info(f"[Rotate to Goal] Heading error: {error_angle:.2f}")
        
        # 허용 오차 검사 및 제어 로직
        if abs(error_angle) > self.angle_tolerance:
            # 오차가 허용 범위를 벗어남 → PID 제어 적용
            angular_correction = self.angular_pid.update(error_angle)
            twist_msg.angular.z = angular_correction    # 각속도 명령
            twist_msg.linear.x = 0.0                    # 회전 중에는 이동 금지
        else:
            # 방향 정렬 완료 → 다음 단계로 전환
            twist_msg.angular.z = 0.0                   # 회전 정지
            twist_msg.linear.x = 0.0                    # 이동도 정지
            self.state = "move_to_goal"                 # 상태 전환: 이동 단계
            self.get_logger().info("Heading aligned. Switching to move_to_goal state.")
            self.publish_state()                        # 상태 변화 알림
        
        return twist_msg

    def handle_move_to_goal(self, current_pose):
        """
        2단계: 목표 위치로 이동 제어 (Move to Goal Position)
        
        터틀을 목표 위치로 이동시키는 단계입니다. 선속도와 각속도를 동시에 제어하여
        효율적인 이동과 방향 보정을 수행합니다. pose_dual_controller와 유사하지만
        상태 기반으로 더 체계적으로 관리됩니다.
        
        제어 전략:
        - 듀얼 PID 제어: 거리 오차(선속도) + 방향 오차(각속도)
        - 로봇 좌표계 기준 거리 오차 계산 (signed distance)
        - 지속적인 방향 보정으로 직선 경로 유지
        - 목표 위치 도달 시 다음 단계로 전환
        
        수학적 배경:
        distance_error = dx*cos(θ) + dy*sin(θ) (벡터 내적)
        - 양수: 목표가 앞쪽 (전진 필요)
        - 음수: 목표가 뒤쪽 (후진 필요)
        
        Args:
            current_pose (Pose): 터틀의 현재 위치 및 방향
            
        Returns:
            Twist: 속도 명령 (linear.x와 angular.z 모두 사용)
        """
        twist_msg = Twist()
        error_msg = Float64()
        
        # 위치 벡터 계산 (목표 - 현재)
        dx = self.goal_pose.x - current_pose.x
        dy = self.goal_pose.y - current_pose.y
        
        # 로봇 좌표계 기준 signed distance error 계산 (핵심!)
        # 터틀이 바라보는 방향을 기준으로 한 거리 오차
        # - 양수: 목표가 앞쪽 → 전진
        # - 음수: 목표가 뒤쪽 → 후진 (일반적으로 발생하지 않음)
        distance_error = dx * math.cos(current_pose.theta) + dy * math.sin(current_pose.theta)
        
        # 거리 오차 정보 발행 (모니터링용)
        error_msg.data = distance_error
        self.error_publisher.publish(error_msg)
        self.get_logger().info(f"[Move to Goal] Distance error: {distance_error:.2f}")
        
        # 거리 허용 오차 검사 및 제어 로직
        if abs(distance_error) > self.distance_tolerance:
            # 목표 위치에 도달하지 못함 → 듀얼 PID 제어 적용
            
            # 1. 선속도 제어 (거리 오차 기반)
            linear_correction = self.linear_pid.update(distance_error)
            twist_msg.linear.x = linear_correction
            
            # 2. 각속도 제어 (방향 오차 기반) - 경로 보정
            # 이동 중에도 지속적으로 목표 방향을 향해 조정
            desired_heading = math.atan2(self.goal_pose.y - current_pose.y,
                                         self.goal_pose.x - current_pose.x)
            angle_error = normalize_angle(desired_heading - current_pose.theta)
            angular_correction = self.angular_pid.update(angle_error)
            twist_msg.angular.z = angular_correction
            
        else:
            # 목표 위치 도달 완료 → 다음 단계로 전환
            twist_msg.linear.x = 0.0                    # 이동 정지
            twist_msg.angular.z = 0.0                   # 회전 정지
            self.state = "rotate_to_final"              # 상태 전환: 최종 방향 조정
            self.get_logger().info("Position reached. Switching to rotate_to_final state.")
            self.publish_state()                        # 상태 변화 알림
        
        return twist_msg

    def handle_rotate_to_final(self, current_pose):
        """
        3단계: 최종 방향으로 회전 제어 (Rotate to Final Orientation)
        
        목표 위치에 도달한 후 최종 목표 방향으로 터틀을 회전시키는 단계입니다.
        이 단계를 통해 터틀이 정확한 자세(위치 + 방향)로 목표를 달성합니다.
        
        제어 전략:
        - 각속도 제어만 사용 (선속도 = 0)
        - 최종 목표 방향과의 오차 계산
        - 각도 오차를 PID로 보정
        - 허용 오차 내 도달 시 목표 완전 달성
        
        이 단계의 중요성:
        - 로봇의 정밀한 자세 제어
        - 다음 작업을 위한 올바른 방향 설정
        - 완전한 목표 달성의 마지막 단계
        
        Args:
            current_pose (Pose): 터틀의 현재 위치 및 방향
            
        Returns:
            Twist: 속도 명령 (angular.z만 사용, linear.x = 0)
        """
        twist_msg = Twist()
        error_msg = Float64()
        
        # 최종 방향 오차 계산 (목표 theta - 현재 theta)
        final_error = normalize_angle(self.goal_pose.theta - current_pose.theta)
        
        # 방향 오차 정보 발행 (모니터링용)
        error_msg.data = final_error
        self.error_publisher.publish(error_msg)
        self.get_logger().info(f"[Rotate to Final] Orientation error: {final_error:.2f}")
        
        # 방향 허용 오차 검사 및 제어 로직
        if abs(final_error) > self.angle_tolerance:
            # 최종 방향에 도달하지 못함 → PID 제어 적용
            angular_correction = self.angular_pid.update(final_error)
            twist_msg.angular.z = angular_correction    # 각속도 명령
            twist_msg.linear.x = 0.0                    # 회전 중에는 이동 금지
        else:
            # 최종 방향 달성 완료 → 목표 완전 달성!
            twist_msg.angular.z = 0.0                   # 회전 정지
            twist_msg.linear.x = 0.0                    # 이동도 정지
            self.state = "goal_reached"                 # 상태 전환: 목표 달성
            self.get_logger().info("Final orientation reached. Goal achieved.")
            self.publish_state()                        # 상태 변화 알림
        
        return twist_msg

    def handle_goal_reached(self):
        """
        4단계: 목표 달성 상태 (Goal Reached State)
        
        모든 제어 목표를 달성한 후의 상태입니다. 터틀을 정지 상태로 유지하며
        새로운 목표가 설정될 때까지 대기합니다.
        
        특징:
        - 모든 속도 명령 = 0 (완전 정지)
        - 안정적인 목표 유지
        - 새 목표 수신 시까지 대기
        
        Returns:
            Twist: 정지 명령 (linear.x = 0, angular.z = 0)
        """
        twist_msg = Twist()
        twist_msg.linear.x = 0.0     # 선속도 정지
        twist_msg.angular.z = 0.0    # 각속도 정지
        return twist_msg

# =============================================================================
# 메인 함수 (프로그램 진입점)
# =============================================================================

def main(args=None):
    """
    고급 터틀 제어기 메인 함수
    
    이 함수는 상태 기반 터틀 제어기의 생명주기를 관리합니다.
    다른 제어기들과 달리 예외 처리를 포함하여 더 안정적인 실행을 보장합니다.
    
    실행 과정:
    1. ROS2 시스템 초기화
    2. 터틀 제어기 노드 생성
    3. 무한 루프 실행 (키보드 인터럽트까지)
    4. 정리 작업 및 시스템 종료
    
    Args:
        args: 명령행 인수 (일반적으로 None)
    """
    # ROS2 클라이언트 라이브러리 초기화
    rclpy.init(args=args)
    
    # 터틀 제어기 노드 생성
    node = TurtleGoalController()
    
    try:
        # 노드 무한 실행 (토픽 콜백 처리)
        # Ctrl+C (KeyboardInterrupt) 시까지 계속 실행
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 사용자가 Ctrl+C로 종료 요청
        node.get_logger().info("Node interrupted")
    finally:
        # 정리 작업 (예외 발생 여부와 관계없이 실행)
        node.destroy_node()    # 노드 리소스 해제
        rclpy.shutdown()       # ROS2 시스템 종료

# =============================================================================
# 스크립트 실행 진입점
# =============================================================================

if __name__ == '__main__':
    """
    스크립트가 직접 실행될 때 main 함수 호출
    
    실행 방법:
    1. python3 move_turtle.py
    2. ros2 run controller_tutorials move_turtle
    
    실행 전 준비사항:
    1. turtlesim_node 실행
    2. goal_pose 토픽 발행 (수동 또는 GUI 모니터 사용)
    
    예시 목표 설정:
    ros2 topic pub /goal_pose turtlesim/msg/Pose "{x: 8.0, y: 6.0, theta: 1.57}"
    """
    main()