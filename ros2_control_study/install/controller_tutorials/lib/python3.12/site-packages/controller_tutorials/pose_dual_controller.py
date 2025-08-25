#!/usr/bin/env python3
"""
거북이 듀얼 PID 위치 제어 노드 (Turtle Dual PID Pose Controller)

이 파일은 2개의 PID 제어기를 사용하여 turtlesim 거북이를 특정 위치(x,y)와 방향(theta)으로 
동시에 제어하는 가장 고급 형태의 제어 노드입니다.

제어기 진화 과정:
┌─────────────────┬──────────────────┬──────────────────┬────────────────────┐
│   파일명        │   제어 방식      │   제어 대상      │      복잡도        │
├─────────────────┼──────────────────┼──────────────────┼────────────────────┤
│ simple_rotate   │   Bang-Bang      │   각도만 (θ)     │   단순 (1D)        │
│ control_rotate  │   단일 PID       │   각도만 (θ)     │   중간 (1D)        │
│ pose_dual_ctrl  │   듀얼 PID       │ 위치+각도 (x,y,θ)│   복잡 (2D+θ)      │
└─────────────────┴──────────────────┴──────────────────┴────────────────────┘

주요 특징:
1. 이중 PID 제어: 선속도용 PID + 각속도용 PID
2. 2D 위치 제어: 목표 위치(x,y)로 이동하면서 동시에 방향 조정
3. 벡터 기반 제어: 현재 위치에서 목표까지의 벡터 계산
4. 동적 방향 조정: 이동 중에 목표 방향을 향해 지속적으로 회전
5. 독립적 허용오차: 거리와 각도에 대해 각각 다른 허용오차 적용

제어 알고리즘:
1. Linear PID (전진/후진 제어):
   - 입력: 거리 오차 (목표까지의 거리)
   - 출력: 선속도 (linear.x)
   - 목적: 목표 위치에 도달

2. Angular PID (회전 제어):
   - 입력: 방향 오차 (목표 방향 - 현재 방향)
   - 출력: 각속도 (angular.z)
   - 목적: 목표를 향해 방향 조정

수학적 배경:
- 거리 계산: distance = √((x₂-x₁)² + (y₂-y₁)²)
- 방향 계산: θ_desired = atan2(y₂-y₁, x₂-x₁)
- 방향 오차: θ_error = normalize_angle(θ_desired - θ_current)

토픽 구조:
- 구독(Subscribe):
  * turtle1/pose (turtlesim/msg/Pose): 거북이의 현재 상태
  * goal_pose (turtlesim/msg/Pose): 목표 위치와 방향
- 발행(Publish):
  * turtle1/cmd_vel (geometry_msgs/msg/Twist): 속도 명령
  * error (std_msgs/msg/Float64): 현재 거리 오차

파라미터 (총 12개, 모두 동적 조정 가능):
Linear PID:
- P_linear, I_linear, D_linear: 선속도 PID 게인
- max_linear, min_linear: 선속도 출력 제한
- distance_tolerance: 목표 도달 허용 거리 오차

Angular PID:  
- P_angular, I_angular, D_angular: 각속도 PID 게인
- max_angular, min_angular: 각속도 출력 제한
- angular_tolerance: 방향 허용 오차

사용법:
1. turtlesim_node 실행
2. 이 노드 실행
3. 목표 설정: ros2 topic pub /goal_pose turtlesim/msg/Pose "{x: 5.0, y: 7.0, theta: 1.57}"
4. PID 튜닝: ros2 param set /turtle_dual_pid_controller P_linear 2.0

작성자: [작성자명]
날짜: [작성일]
"""

# ROS2 기본 라이브러리들 import
import rclpy  # ROS2 Python 클라이언트 라이브러리
from rclpy.node import Node  # ROS2 노드의 기본 클래스
from turtlesim.msg import Pose  # turtlesim 패키지의 거북이 자세 정보 (x, y, theta 포함)
from geometry_msgs.msg import Twist  # 속도 명령을 위한 표준 메시지 (선속도, 각속도)
from std_msgs.msg import Float64  # 실수 데이터 전송용 메시지 (오차 정보)
import math  # 수학 연산 (삼각함수, 벡터 계산 등)
from controller_tutorials.control_apps import PID  # 우리가 만든 PID 제어기 클래스
from rcl_interfaces.msg import SetParametersResult  # 파라미터 동적 변경 결과

def normalize_angle(angle):
    """
    각도 정규화 함수
    
    임의의 각도를 -π ~ π 범위로 정규화합니다.
    이는 각도 제어에서 최단 경로로 회전하기 위해 필수적입니다.
    
    예시:
        - 입력: 3π/2 (270°) → 출력: -π/2 (-90°)
        - 입력: 5π/4 (225°) → 출력: -3π/4 (-135°)
    
    Args:
        angle (float): 정규화할 각도 (라디안)
        
    Returns:
        float: -π ~ π 범위로 정규화된 각도 (라디안)
    """
    return math.atan2(math.sin(angle), math.cos(angle))

class TurtleDualPIDController(Node):
    """
    거북이 듀얼 PID 제어 클래스
    
    이 클래스는 2개의 독립적인 PID 제어기를 사용하여 거북이의 위치와 방향을 동시에 제어합니다.
    이전 파일들과 달리 2차원 위치 제어가 가능한 가장 고급 형태의 제어기입니다.
    
    제어 아키텍처:
    ┌─────────────┐    ┌──────────────┐    ┌─────────────┐
    │  Goal Pose  │───▶│ Distance PID │───▶│ Linear Vel  │
    │   (x,y,θ)   │    │   Controller │    │   (cmd_vel) │
    └─────────────┘    └──────────────┘    └─────────────┘
           │            ┌──────────────┐    ┌─────────────┐
           └───────────▶│ Angular PID  │───▶│ Angular Vel │
                        │   Controller │    │   (cmd_vel) │
                        └──────────────┘    └─────────────┘
    
    주요 특징:
    - 독립적 이중 제어: 선속도와 각속도를 각각 별도의 PID로 제어
    - 동시 제어: 목표 위치로 이동하면서 동시에 방향 조정
    - 벡터 기반: 2D 벡터 계산을 통한 정밀한 위치 제어
    - 12개 파라미터: 각 PID마다 독립적인 게인 및 제한값 설정
    """
    
    def __init__(self):
        """
        듀얼 PID 제어 노드 초기화
        
        초기화 과정:
        1. Linear PID 파라미터 선언 및 설정 (전진/후진 제어용)
        2. Angular PID 파라미터 선언 및 설정 (회전 제어용)
        3. 두 개의 PID 제어기 객체 생성 및 초기화
        4. 목표 위치 변수 초기화
        5. ROS2 토픽 구독자/발행자 설정
        6. 파라미터 동적 변경 콜백 등록
        """
        # 부모 클래스(Node) 초기화
        super().__init__('turtle_dual_pid_controller')
        
        # =============================================================================
        # Linear PID 파라미터 선언 (전진/후진 제어용)
        # =============================================================================
        # 거북이가 목표 위치에 도달하기 위한 선속도를 제어하는 PID 파라미터들
        self.declare_parameter('P_linear', 1.0)         # 비례 게인: 거리 오차에 대한 즉각적 반응
        self.declare_parameter('I_linear', 0.0)         # 적분 게인: 누적 거리 오차 보정
        self.declare_parameter('D_linear', 0.0)         # 미분 게인: 접근 속도 조절 (오버슈트 방지)
        self.declare_parameter('max_linear', 5.0)       # 최대 선속도 제한 [m/s]
        self.declare_parameter('min_linear', -5.0)      # 최소 선속도 제한 (후진 가능)
        self.declare_parameter('distance_tolerance', 0.1)  # 목표 도달 허용 거리 오차 [m]
        
        # Linear PID 파라미터 값 읽기
        P_linear = self.get_parameter('P_linear').value
        I_linear = self.get_parameter('I_linear').value
        D_linear = self.get_parameter('D_linear').value
        max_linear = self.get_parameter('max_linear').value
        min_linear = self.get_parameter('min_linear').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        
        # =============================================================================
        # Angular PID 파라미터 선언 (회전 제어용)
        # =============================================================================
        # 거북이가 목표 방향을 향하도록 하는 각속도를 제어하는 PID 파라미터들
        self.declare_parameter('P_angular', 5.0)        # 비례 게인: 방향 오차에 대한 즉각적 반응
        self.declare_parameter('I_angular', 0.0)        # 적분 게인: 누적 방향 오차 보정
        self.declare_parameter('D_angular', 0.0)        # 미분 게인: 회전 속도 조절 (진동 방지)
        self.declare_parameter('max_angular', 5.0)      # 최대 각속도 제한 [rad/s]
        self.declare_parameter('min_angular', -5.0)     # 최소 각속도 제한 (양방향 회전)
        self.declare_parameter('angular_tolerance', 0.01)  # 방향 허용 오차 [rad]
        
        # Angular PID 파라미터 값 읽기
        P_angular = self.get_parameter('P_angular').value
        I_angular = self.get_parameter('I_angular').value
        D_angular = self.get_parameter('D_angular').value
        max_angular = self.get_parameter('max_angular').value
        min_angular = self.get_parameter('min_angular').value
        self.angular_tolerance = self.get_parameter('angular_tolerance').value
        
        # =============================================================================
        # 듀얼 PID 제어기 객체 생성 및 초기화
        # =============================================================================
        # 1. Linear PID 제어기 생성 (전진/후진 제어용)
        self.linear_pid = PID()
        self.linear_pid.P = P_linear            # 거리 오차에 대한 비례 반응
        self.linear_pid.I = I_linear            # 누적 거리 오차 보정
        self.linear_pid.D = D_linear            # 접근 속도 조절 (급제동 방지)
        self.linear_pid.max_state = max_linear  # 최대 전진 속도 제한
        self.linear_pid.min_state = min_linear  # 최대 후진 속도 제한
        
        # 2. Angular PID 제어기 생성 (회전 제어용)
        self.angular_pid = PID()
        self.angular_pid.P = P_angular            # 방향 오차에 대한 비례 반응
        self.angular_pid.I = I_angular            # 누적 방향 오차 보정  
        self.angular_pid.D = D_angular            # 회전 속도 조절 (진동 방지)
        self.angular_pid.max_state = max_angular  # 최대 반시계방향 각속도 제한
        self.angular_pid.min_state = min_angular  # 최대 시계방향 각속도 제한
        
        # =============================================================================
        # 목표 위치 변수 초기화
        # =============================================================================
        # 목표 위치 및 방향 (goal_pose 토픽에서 업데이트됨)
        # 초기값은 원점으로 설정하되, 실제로는 첫 번째 pose_callback에서 현재 위치로 설정됨
        self.goal_pose = Pose()
        self.goal_pose.x = 0.0      # 목표 x 좌표 [m]
        self.goal_pose.y = 0.0      # 목표 y 좌표 [m]  
        self.goal_pose.theta = 0.0  # 목표 방향각 [rad]
        
        # 초기 목표 설정 플래그
        # 외부에서 goal_pose가 설정되기 전까지는 현재 위치를 목표로 유지하여 거북이가 움직이지 않도록 함
        self.initial_goal_set = False
        
        # =============================================================================
        # ROS2 토픽 구독자(Subscriber) 설정
        # =============================================================================
        # 1. 거북이 현재 자세 정보 구독
        self.pose_subscriber = self.create_subscription(
            Pose,                    # 메시지 타입: 거북이의 위치(x,y)와 방향(theta)
            'turtle1/pose',          # 토픽 이름: turtlesim이 발행하는 거북이 상태
            self.pose_callback,      # 콜백 함수: 자세 정보가 올 때마다 듀얼 PID 제어 수행
            10                       # 큐 크기
        )
        
        # 2. 목표 위치 명령 구독 (이전 파일들과의 차이점)
        # simple_rotate.py, control_rotate.py: Float64 타입의 목표 각도만 받음
        # pose_dual_controller.py: Pose 타입의 목표 위치+방향을 받음 (x, y, theta 모두 포함)
        self.goal_subscriber = self.create_subscription(
            Pose,                    # 메시지 타입: 목표 위치(x,y)와 방향(theta) 모두 포함
            'goal_pose',             # 토픽 이름: 사용자 정의 목표 설정 토픽
            self.goal_callback,      # 콜백 함수: 새로운 목표 위치가 설정될 때 호출
            10                       # 큐 크기
        )
        
        # =============================================================================
        # ROS2 토픽 발행자(Publisher) 설정
        # =============================================================================
        # 1. 거북이 속도 명령 발행 (듀얼 제어 출력)
        self.cmd_vel_pub = self.create_publisher(
            Twist,                   # 메시지 타입: linear.x (전진/후진) + angular.z (회전)
            'turtle1/cmd_vel',       # 토픽 이름: turtlesim이 구독하는 속도 명령 토픽
            10                       # 큐 크기
        )
        
        # 2. 거리 오차 정보 발행 (모니터링용)
        self.error_publisher = self.create_publisher(
            Float64,                 # 메시지 타입: 현재 목표까지의 거리 오차
            'error',                 # 토픽 이름: 오차 모니터링을 위한 토픽
            10                       # 큐 크기
        )
        
        # =============================================================================
        # 파라미터 동적 변경 콜백 등록
        # =============================================================================
        # 12개 파라미터 (Linear PID 6개 + Angular PID 6개)의 실시간 조정을 위한 콜백 등록
        self.add_on_set_parameters_callback(self.parameter_callback)
    
    def parameter_callback(self, params):
        """
        듀얼 PID 파라미터 동적 변경 콜백 함수
        
        총 12개의 파라미터를 실시간으로 조정할 수 있는 콜백 함수입니다.
        이전 파일들보다 훨씬 많은 파라미터를 관리하므로 체계적인 튜닝이 가능합니다.
        
        파라미터 구조:
        ┌─────────────────┬───────────────────┬─────────────────────────────┐
        │  PID 컨트롤러     │       파라미터 타입  │            조정 효과          │
        ├─────────────────┼───────────────────┼─────────────────────────────┤
        │ Linear PID      │ P_linear          │ 거리 오차 반응 속도         │
        │ (전진/후진)       │ I_linear          │ 정상상태 거리 오차 제거     │
        │                 │ D_linear          │ 접근 시 급제동 방지         │
        │                 │ max/min_linear    │ 선속도 제한                 │
        │                 │ distance_tolerance│ 목표 도달 허용 거리         │
        ├─────────────────┼───────────────────┼─────────────────────────────┤
        │ Angular PID     │ P_angular         │ 방향 오차 반응 속도         │
        │ (회전)          │ I_angular         │ 정상상태 방향 오차 제거     │
        │                 │ D_angular         │ 회전 시 진동 억제           │
        │                 │ max/min_angular   │ 각속도 제한                 │
        │                 │ angular_tolerance │ 방향 허용 오차              │
        └─────────────────┴───────────────────┴─────────────────────────────┘
        
        사용 예시:
            # Linear PID 튜닝
            ros2 param set /turtle_dual_pid_controller P_linear 2.0
            ros2 param set /turtle_dual_pid_controller distance_tolerance 0.05
            
            # Angular PID 튜닝  
            ros2 param set /turtle_dual_pid_controller P_angular 8.0
            ros2 param set /turtle_dual_pid_controller D_angular 0.1
        
        Args:
            params: 변경된 파라미터들의 리스트
            
        Returns:
            SetParametersResult: 파라미터 변경 성공/실패 결과
        """
        for param in params:
            
            # =============================================================================
            # Linear PID 파라미터 업데이트 (전진/후진 제어용)
            # =============================================================================
            if param.name == 'P_linear':
                self.linear_pid.P = param.value
                self.get_logger().info(f"Updated linear PID P: {param.value}")
            elif param.name == 'I_linear':
                self.linear_pid.I = param.value
                self.get_logger().info(f"Updated linear PID I: {param.value}")
            elif param.name == 'D_linear':
                self.linear_pid.D = param.value
                self.get_logger().info(f"Updated linear PID D: {param.value}")
            elif param.name == 'max_linear':
                self.linear_pid.max_state = param.value
                self.get_logger().info(f"Updated linear PID max_linear: {param.value}")
            elif param.name == 'min_linear':
                self.linear_pid.min_state = param.value
                self.get_logger().info(f"Updated linear PID min_linear: {param.value}")
            elif param.name == 'distance_tolerance':
                self.distance_tolerance = param.value
                self.get_logger().info(f"Updated distance_tolerance: {param.value}")
                
            # =============================================================================
            # Angular PID 파라미터 업데이트 (회전 제어용)
            # =============================================================================
            elif param.name == 'P_angular':
                self.angular_pid.P = param.value
                self.get_logger().info(f"Updated angular PID P: {param.value}")
            elif param.name == 'I_angular':
                self.angular_pid.I = param.value
                self.get_logger().info(f"Updated angular PID I: {param.value}")
            elif param.name == 'D_angular':
                self.angular_pid.D = param.value
                self.get_logger().info(f"Updated angular PID D: {param.value}")
            elif param.name == 'max_angular':
                self.angular_pid.max_state = param.value
                self.get_logger().info(f"Updated angular PID max_angular: {param.value}")
            elif param.name == 'min_angular':
                self.angular_pid.min_state = param.value
                self.get_logger().info(f"Updated angular PID min_angular: {param.value}")
            elif param.name == 'angular_tolerance':
                self.angular_tolerance = param.value
                self.get_logger().info(f"Updated angular_tolerance: {param.value}")
                
        # 모든 파라미터 변경이 성공했음을 반환
        return SetParametersResult(successful=True)
    
    def goal_callback(self, msg):
        """
        목표 위치 수신 콜백 함수
        
        goal_pose 토픽으로부터 새로운 목표 위치와 방향을 받을 때마다 호출됩니다.
        이전 파일들과 달리 3차원 목표(x, y, theta)를 모두 받을 수 있습니다.
        
        이전 파일들과의 차이점:
        - simple_rotate.py: Float64 (theta만)
        - control_rotate.py: Float64 (theta만)  
        - pose_dual_controller.py: Pose (x, y, theta 모두)
        
        Args:
            msg (Pose): 목표 위치와 방향 정보
                       - msg.x: 목표 x 좌표 [m]
                       - msg.y: 목표 y 좌표 [m]
                       - msg.theta: 목표 방향각 [rad]
        """
        # 새로운 목표 위치 전체 업데이트
        self.goal_pose = msg
        self.get_logger().info(
            f"Received new goal_pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}")
    
    def pose_callback(self, msg):
        """
        거북이 자세 정보 수신 및 듀얼 PID 제어 수행 함수 (최고급 제어 로직)
        
        turtlesim에서 발행하는 거북이의 현재 자세 정보를 받아서 듀얼 PID 제어를 수행합니다.
        이 함수는 시리즈 중 가장 복잡한 제어 로직을 구현하며, 2D 위치 제어의 핵심입니다.
        
        제어 방식 진화:
        1. simple_rotate.py: Bang-Bang (각도만)
        2. control_rotate.py: 단일 PID (각도만)
        3. pose_dual_controller.py: 듀얼 PID (위치 + 각도)
        
        듀얼 PID 제어 과정:
        1. 초기 목표 설정 (안전 장치)
        2. Linear PID: 거리 오차 → 선속도
        3. Angular PID: 방향 오차 → 각속도
        4. 동시 제어 출력 및 상태 모니터링
        
        Args:
            msg (Pose): 거북이의 현재 자세 정보
                       - msg.x, msg.y: 현재 위치 좌표 [m]
                       - msg.theta: 현재 방향각 [rad]
        """
        current_pose = msg
        
        # =============================================================================
        # 1. 초기 목표 설정 (안전 장치)
        # =============================================================================
        # 외부에서 goal_pose가 아직 설정되지 않았다면 현재 위치를 목표로 설정
        # 이를 통해 목표가 없을 때 거북이가 무작정 움직이는 것을 방지
        if not self.initial_goal_set:
            self.goal_pose = msg
            self.initial_goal_set = True
            self.get_logger().info("Initial goal_pose set to current turtle pose.")
        
        # 제어 출력 메시지 초기화
        twist_msg = Twist()
        
        # =============================================================================
        # 2. Linear PID 제어 (전진/후진 제어)
        # =============================================================================
        # 2-1. 위치 벡터 계산
        dx = self.goal_pose.x - current_pose.x  # x 방향 거리 차이
        dy = self.goal_pose.y - current_pose.y  # y 방향 거리 차이
        
        # 2-2. 로봇 좌표계 기준 거리 오차 계산 (핵심 수학!)
        # 로봇이 현재 바라보는 방향을 기준으로 한 signed distance error
        # - 양수: 목표가 앞쪽 (전진 필요)
        # - 음수: 목표가 뒤쪽 (후진 필요)
        # 공식: d_error = dx*cos(θ) + dy*sin(θ) (벡터 내적)
        distance_error = dx * math.cos(current_pose.theta) + dy * math.sin(current_pose.theta)
        
        # 2-3. 거리 오차 발행 (모니터링용)
        error_msg = Float64()
        error_msg.data = distance_error
        self.error_publisher.publish(error_msg)
        self.get_logger().info(f"[Move to Goal] Distance error: {distance_error:.2f}")
        
        # 2-4. Linear PID 제어 출력 계산
        if abs(distance_error) < self.distance_tolerance:
            # 목표에 충분히 가까우면 정지
            twist_msg.linear.x = 0.0
        else:
            # Linear PID로 선속도 계산 (거리 오차 → 선속도)
            linear_correction = self.linear_pid.update(distance_error)
            twist_msg.linear.x = linear_correction
        
        # =============================================================================
        # 3. Angular PID 제어 (회전 제어)
        # =============================================================================
        # 3-1. 목표 방향 계산
        # 현재 위치에서 목표 위치를 향하는 방향각 계산
        desired_heading = math.atan2(self.goal_pose.y - current_pose.y,
                                    self.goal_pose.x - current_pose.x)
        
        # 3-2. 방향 오차 계산 및 정규화
        # 현재 방향과 목표 방향의 차이를 -π ~ π 범위로 정규화
        angle_error = normalize_angle(desired_heading - current_pose.theta)
        
        # 3-3. Angular PID 제어 출력 계산
        if abs(angle_error) < self.angular_tolerance:
            # 방향이 충분히 정확하면 회전 정지
            twist_msg.angular.z = 0.0
        else:
            # Angular PID로 각속도 계산 (방향 오차 → 각속도)
            angular_correction = self.angular_pid.update(angle_error)
            twist_msg.angular.z = angular_correction
        
        # =============================================================================
        # 4. 듀얼 제어 출력 발행 및 상태 모니터링
        # =============================================================================
        # 계산된 선속도와 각속도를 동시에 발행 (듀얼 제어의 핵심!)
        self.cmd_vel_pub.publish(twist_msg)
        
        # 상세한 상태 정보 로그 출력 (디버깅 및 성능 분석)
        self.get_logger().info(
            f"Current Pose: (x={current_pose.x:.2f}, y={current_pose.y:.2f}, theta={current_pose.theta:.2f}), " +
            f"Goal Pose: (x={self.goal_pose.x:.2f}, y={self.goal_pose.y:.2f}, theta={self.goal_pose.theta:.2f}), " +
            f"Angle error: {angle_error:.2f}"
        )

# =============================================================================
# 메인 함수 및 프로그램 진입점
# =============================================================================

def main(args=None):
    """
    메인 함수 - 듀얼 PID 제어 노드를 실행하고 관리
    
    이 함수는 시리즈 중 가장 고급 형태의 제어 노드를 실행합니다.
    simple_rotate.py와 control_rotate.py보다 훨씬 복잡한 제어를 수행하지만,
    기본적인 ROS2 노드 관리 구조는 동일합니다.
    
    제어 복잡도 비교:
    - simple_rotate.py: 1D 제어 (각도만)
    - control_rotate.py: 1D 제어 (각도만, PID)
    - pose_dual_controller.py: 3D 제어 (x, y, theta, 듀얼 PID)
    
    Args:
        args: 명령줄 인수 (일반적으로 None)
    
    실행 흐름:
    1. ROS2 시스템 초기화
    2. 듀얼 PID 제어 노드 객체 생성 (12개 파라미터 설정 포함)
    3. 노드 실행 (듀얼 PID 제어 루프)
    4. 예외 처리 및 정리
    
    사용법:
        python3 pose_dual_controller.py
        또는
        ros2 run controller_tutorials pose_dual_controller
        
    이후 목표 설정:
        ros2 topic pub /goal_pose turtlesim/msg/Pose "{x: 5.0, y: 7.0, theta: 1.57}"
    """
    # ROS2 시스템 초기화 (필수)
    rclpy.init(args=args)
    
    # 듀얼 PID 제어 노드 객체 생성
    # 이때 __init__ 메서드에서 두 개의 PID 제어기와 12개 파라미터가 모두 설정됨
    node = TurtleDualPIDController()
    
    try:
        # 노드 실행 시작 (무한 루프)
        # 듀얼 PID 제어기가 지속적으로 거북이의 자세를 모니터링하고 위치+방향을 동시 제어
        # pose_callback이 거북이 자세 정보가 올 때마다 자동으로 호출되어 복잡한 제어 수행
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        # Ctrl+C로 프로그램 종료 시 실행
        node.get_logger().info("Node interrupted.")
        
    finally:
        # 프로그램 종료 시 정리 작업
        # 두 개의 PID 제어기와 모든 토픽, 파라미터 정리
        node.destroy_node()  # 노드 객체 해제 (듀얼 PID 제어기 포함)
        rclpy.shutdown()     # ROS2 시스템 종료

# Python 스크립트가 직접 실행될 때만 main() 함수 호출
# (다른 모듈에서 import될 때는 실행되지 않음)
if __name__ == '__main__':
    main()