#!/usr/bin/env python3
"""
거북이 PID 회전 제어 노드 (Turtle PID Angular Controller)

이 파일은 PID 제어기를 사용하여 turtlesim 거북이를 특정 방향으로 회전시키는 고급 제어 노드입니다.
simple_rotate.py의 Bang-Bang 제어 방식을 PID 제어로 개선한 버전입니다.

Bang-Bang vs PID 제어 비교:
┌─────────────────┬──────────────────┬────────────────────┐
│     특성        │   Bang-Bang      │       PID          │
├─────────────────┼──────────────────┼────────────────────┤
│ 제어 방식       │ On/Off 방식      │ 연속적 제어        │
│ 응답 특성       │ 빠르지만 진동    │ 부드럽고 안정적    │
│ 정확도          │ 허용오차 범위    │ 높은 정밀도        │
│ 오버슈트        │ 클 수 있음       │ 조정 가능          │
│ 구현 복잡도     │ 간단             │ 약간 복잡          │
└─────────────────┴──────────────────┴────────────────────┘

주요 개선사항:
1. 부드러운 제어: 급격한 속도 변화 없이 목표에 접근
2. 높은 정확도: PID의 적분항으로 정상상태 오차 제거
3. 안정성: 미분항으로 오버슈트 억제
4. 조정 가능성: P, I, D 게인을 통한 세밀한 성능 튜닝

토픽 구조:
- 구독(Subscribe):
  * turtle1/pose (turtlesim/msg/Pose): 거북이의 현재 상태
  * goal_theta (std_msgs/msg/Float64): 목표 방향각 (라디안)
- 발행(Publish):
  * turtle1/cmd_vel (geometry_msgs/msg/Twist): 속도 명령
  * error (std_msgs/msg/Float64): 현재 제어 오차

파라미터 (동적 조정 가능):
- P (double): 비례 게인 [기본값: 1.0]
- I (double): 적분 게인 [기본값: 0.0]  
- D (double): 미분 게인 [기본값: 0.0]
- max_state (double): 최대 제어 출력 [기본값: 5.0]
- min_state (double): 최소 제어 출력 [기본값: -5.0]
- tolerance (double): 목표 도달 허용 오차 [기본값: 0.01]

PID 튜닝 가이드:
1. P 증가: 빠른 응답, 하지만 진동 증가
2. I 증가: 정상상태 오차 감소, 하지만 오버슈트 증가
3. D 증가: 오버슈트 감소, 하지만 노이즈에 민감

사용법:
1. turtlesim_node 실행
2. 이 노드 실행  
3. PID 게인 조정: ros2 param set /turtle_pid_controller P 2.0
4. 목표 설정: ros2 topic pub /goal_theta std_msgs/msg/Float64 "data: 1.57"

작성자: [작성자명]
날짜: [작성일]
"""

# ROS2 기본 라이브러리들 import
import rclpy  # ROS2 Python 클라이언트 라이브러리
from rclpy.node import Node  # ROS2 노드의 기본 클래스
from turtlesim.msg import Pose  # turtlesim 패키지의 거북이 자세 정보
from geometry_msgs.msg import Twist  # 속도 명령을 위한 표준 메시지
import math  # 수학 연산 (각도 정규화 등)
from controller_tutorials.control_apps import PID  # 우리가 만든 PID 제어기 클래스
from rcl_interfaces.msg import SetParametersResult  # 파라미터 동적 변경 결과
from std_msgs.msg import Float64  # 실수 데이터 전송용 메시지

class TurtlePIDController(Node):
    """
    거북이 PID 제어 클래스
    
    이 클래스는 PID 제어기를 사용하여 거북이의 방향을 제어합니다.
    simple_rotate.py의 Bang-Bang 제어와 달리, 연속적이고 부드러운 제어를 제공합니다.
    
    주요 특징:
    - PID 제어기를 통한 정밀한 각속도 제어
    - 실시간 파라미터 조정 가능 (P, I, D 게인)
    - 제어 출력 제한 기능
    - 목표 도달 시 자동 정지
    """
    
    def __init__(self):
        """
        PID 제어 노드 초기화
        
        1. ROS2 파라미터 선언 및 초기화
        2. PID 제어기 객체 생성 및 설정
        3. 토픽 구독자/발행자 설정
        4. 파라미터 동적 변경 콜백 등록
        """
        # 부모 클래스(Node) 초기화
        super().__init__('turtle_pid_controller')
        
        # =============================================================================
        # ROS2 파라미터 선언 - 실행 중 동적 변경 가능
        # =============================================================================
        # PID 제어기 게인 파라미터
        self.declare_parameter('P', 1.0)         # 비례 게인 (응답 속도 조절)
        self.declare_parameter('I', 0.0)         # 적분 게인 (정상상태 오차 제거)
        self.declare_parameter('D', 0.0)         # 미분 게인 (오버슈트 억제)
        
        # 제어 출력 제한 파라미터
        self.declare_parameter('max_state', 5.0)  # 최대 각속도 [rad/s]
        self.declare_parameter('min_state', -5.0) # 최소 각속도 [rad/s] (실제로는 최소 절댓값)
        
        # 제어 성능 파라미터
        self.declare_parameter('tolerance', 0.01) # 목표 도달 허용 오차 [rad]
        
        # =============================================================================
        # 파라미터 값 읽기 및 변수 할당
        # =============================================================================
        P = self.get_parameter('P').value
        I = self.get_parameter('I').value
        D = self.get_parameter('D').value
        max_state = self.get_parameter('max_state').value
        min_state = self.get_parameter('min_state').value
        self.tolerance = self.get_parameter('tolerance').value

        # =============================================================================
        # PID 제어기 객체 생성 및 초기화
        # =============================================================================
        # control_apps.py에서 정의한 PID 클래스의 인스턴스 생성
        self.pid = PID()
        
        # PID 게인 설정
        self.pid.P = P           # 비례 게인: 현재 오차에 대한 반응 강도
        self.pid.I = I           # 적분 게인: 누적 오차에 대한 반응 강도
        self.pid.D = D           # 미분 게인: 오차 변화율에 대한 반응 강도
        
        # 출력 제한 설정 (거북이 각속도의 물리적 한계)
        self.pid.max_state = max_state  # 최대 각속도 제한
        self.pid.min_state = min_state  # 최소 각속도 제한

        # =============================================================================
        # 제어 변수 초기화
        # =============================================================================
        # 목표 방향각 (goal_theta 토픽에서 업데이트됨)
        self.target_theta = 0.0

        # =============================================================================
        # ROS2 토픽 구독자(Subscriber) 설정
        # =============================================================================
        # 1. 거북이 자세 정보 구독
        self.pose_subscriber = self.create_subscription(
            Pose,                    # 메시지 타입: 거북이의 위치와 방향 정보
            'turtle1/pose',          # 토픽 이름: turtlesim이 발행하는 거북이 상태
            self.pose_callback,      # 콜백 함수: 자세 정보가 올 때마다 PID 제어 수행
            10                       # 큐 크기
        )
        
        # 2. 목표 방향각 명령 구독
        self.goal_subscriber = self.create_subscription(
            Float64,                 # 메시지 타입: 목표 방향각 (라디안)
            'goal_theta',            # 토픽 이름: 사용자 정의 목표 설정 토픽
            self.goal_callback,      # 콜백 함수: 새로운 목표가 설정될 때 호출
            10                       # 큐 크기
        )
        
        # =============================================================================
        # ROS2 토픽 발행자(Publisher) 설정
        # =============================================================================
        # 1. 거북이 속도 명령 발행
        self.publisher = self.create_publisher(
            Twist,                   # 메시지 타입: 선속도와 각속도 명령
            'turtle1/cmd_vel',       # 토픽 이름: turtlesim이 구독하는 속도 명령 토픽
            10                       # 큐 크기
        )
        
        # 2. 제어 오차 정보 발행 (모니터링용)
        self.error_publisher = self.create_publisher(
            Float64,                 # 메시지 타입: 현재 제어 오차 값
            'error',                 # 토픽 이름: 오차 모니터링을 위한 토픽
            10                       # 큐 크기
        )
        
        # =============================================================================
        # 파라미터 동적 변경 콜백 등록
        # =============================================================================
        # 실행 중에 PID 게인 등을 조정할 수 있도록 콜백 함수 등록
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """
        파라미터 동적 변경 콜백 함수
        
        실행 중에 PID 게인이나 기타 파라미터가 변경될 때 자동으로 호출됩니다.
        이를 통해 시스템 동작 중에도 제어 성능을 실시간으로 튜닝할 수 있습니다.
        
        사용 예시:
            ros2 param set /turtle_pid_controller P 2.0
            ros2 param set /turtle_pid_controller D 0.1
            
        Args:
            params: 변경된 파라미터들의 리스트
            
        Returns:
            SetParametersResult: 파라미터 변경 성공/실패 결과
        """
        for param in params:
            # PID 게인 파라미터들
            if param.name == 'P':
                self.pid.P = param.value
                self.get_logger().info(f"Updated PID P: {param.value}")
            elif param.name == 'I':
                self.pid.I = param.value
                self.get_logger().info(f"Updated PID I: {param.value}")
            elif param.name == 'D':
                self.pid.D = param.value
                self.get_logger().info(f"Updated PID D: {param.value}")
                
            # 출력 제한 파라미터들
            elif param.name == 'max_state':
                self.pid.max_state = param.value
                self.get_logger().info(f"Updated PID max_state: {param.value}")
            elif param.name == 'min_state':
                self.pid.min_state = param.value
                self.get_logger().info(f"Updated PID min_state: {param.value}")
                
            # 제어 성능 파라미터
            elif param.name == 'tolerance':
                self.tolerance = param.value
                self.get_logger().info(f"Updated tolerance: {param.value}")
                
        # 모든 파라미터 변경이 성공했음을 반환
        return SetParametersResult(successful=True)

    def goal_callback(self, msg):
        """
        목표 방향각 수신 콜백 함수
        
        goal_theta 토픽으로부터 새로운 목표 방향을 받을 때마다 호출됩니다.
        PID 제어기는 이 목표값을 향해 거북이를 부드럽게 회전시킵니다.
        
        Args:
            msg (Float64): 목표 방향각 (라디안 단위)
                          - 0.0: 동쪽 방향 (→)
                          - π/2: 북쪽 방향 (↑)  
                          - π: 서쪽 방향 (←)
                          - 3π/2: 남쪽 방향 (↓)
        """
        # 새로운 목표 방향각 업데이트
        self.target_theta = msg.data
        self.get_logger().info(f"Received new goal_theta: {self.target_theta:.2f}")

    def pose_callback(self, msg):
        """
        거북이 자세 정보 수신 및 PID 제어 수행 함수 (핵심 제어 로직)
        
        turtlesim에서 발행하는 거북이의 현재 자세 정보를 받아서 PID 제어를 수행합니다.
        simple_rotate.py의 Bang-Bang 제어와 달리, 연속적이고 부드러운 제어 출력을 생성합니다.
        
        PID 제어 과정:
        1. 오차 계산 및 정규화 (-π ~ π)
        2. 오차 정보 발행 (모니터링용)
        3. 목표 도달 검사 (tolerance 기반)
        4. PID 제어 출력 계산 또는 정지
        5. 속도 명령 발행
        
        Args:
            msg (Pose): 거북이의 현재 자세 정보
                       - msg.x, msg.y: 위치 좌표
                       - msg.theta: 현재 방향각 (라디안)
                       - msg.linear_velocity, msg.angular_velocity: 현재 속도
        """
        
        # =============================================================================
        # 1. 오차 계산 및 정규화
        # =============================================================================
        # 목표 방향과 현재 방향의 차이 계산
        error = self.target_theta - msg.theta
        
        # 각도 오차를 -π ~ π 범위로 정규화 (최단 경로 회전을 위함)
        # 예: 목표 10°, 현재 350° → 오차 20° (시계방향) 대신 -340° (반시계방향)으로 계산하지 않도록 함
        error = math.atan2(math.sin(error), math.cos(error))
        
        # =============================================================================
        # 2. 오차 정보 발행 (모니터링/디버깅 용도)
        # =============================================================================
        error_msg = Float64()
        error_msg.data = error
        self.error_publisher.publish(error_msg)
        
        # =============================================================================
        # 3. 제어 출력 계산 (PID 제어 vs Bang-Bang 제어)
        # =============================================================================
        twist_msg = Twist()
        
        # 목표 도달 검사: 오차가 허용 범위 내에 있는지 확인
        if abs(error) < self.tolerance:
            # 목표에 충분히 가까우면 정지 (Bang-Bang과 동일한 조건)
            twist_msg.angular.z = 0.0
            self.get_logger().info("Error within tolerance. Controller stopped.")
        else:
            # PID 제어기를 사용하여 연속적인 제어 출력 계산
            # Bang-Bang 제어와의 차이점:
            # - Bang-Bang: 고정된 각속도 (예: ±1.0 rad/s)
            # - PID: 오차에 비례하여 연속적으로 변하는 각속도
            angular_correction = self.pid.update(error)
            twist_msg.angular.z = angular_correction
            
        # 선속도는 항상 0 (제자리 회전만 수행)
        twist_msg.linear.x = 0.0
        
        # =============================================================================
        # 4. 제어 명령 발행 및 상태 로그
        # =============================================================================
        self.publisher.publish(twist_msg)
        
        # 현재 제어 상태를 로그로 출력 (디버깅 및 성능 분석용)
        self.get_logger().info(
            f"Current theta: {msg.theta:.2f}, "
            f"Target theta: {self.target_theta:.2f}, "
            f"Error: {error:.2f}, "
            f"Control Output: {twist_msg.angular.z:.2f}"
        )

# =============================================================================
# 메인 함수 및 프로그램 진입점
# =============================================================================

def main(args=None):
    """
    메인 함수 - PID 제어 노드를 실행하고 관리
    
    이 함수는 simple_rotate.py의 메인 함수와 동일한 구조이지만,
    TurtlePIDController 클래스를 사용하여 더 정교한 제어를 수행합니다.
    
    Args:
        args: 명령줄 인수 (일반적으로 None)
    
    실행 흐름:
    1. ROS2 시스템 초기화
    2. PID 제어 노드 객체 생성
    3. 노드 실행 (PID 제어 루프)
    4. 예외 처리 및 정리
    
    사용법:
        python3 control_rotate.py
        또는
        ros2 run controller_tutorials control_rotate
    """
    # ROS2 시스템 초기화 (필수)
    rclpy.init(args=args)
    
    # PID 제어 노드 객체 생성
    # 이때 __init__ 메서드에서 모든 설정이 완료됨
    node = TurtlePIDController()
    
    try:
        # 노드 실행 시작 (무한 루프)
        # PID 제어기가 지속적으로 거북이의 자세를 모니터링하고 제어
        # pose_callback이 거북이 자세 정보가 올 때마다 자동으로 호출됨
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        # Ctrl+C로 프로그램 종료 시 실행
        node.get_logger().info("Node interrupted")
        
    finally:
        # 프로그램 종료 시 정리 작업
        node.destroy_node()  # 노드 객체 해제 (토픽, 파라미터, 타이머 등 정리)
        rclpy.shutdown()     # ROS2 시스템 종료

# Python 스크립트가 직접 실행될 때만 main() 함수 호출
# (다른 모듈에서 import될 때는 실행되지 않음)
if __name__ == '__main__':
    main()