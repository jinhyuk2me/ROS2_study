#!/usr/bin/env python3
"""
거북이 방향 제어 노드 (Turtle Angular Controller)

이 파일은 ROS2 turtlesim 패키지의 거북이를 특정 방향으로 회전시키는 제어 노드입니다.

주요 기능:
1. goal_theta 토픽으로부터 목표 방향각을 받음
2. turtlesim의 현재 방향과 목표 방향 사이의 오차를 계산
3. Bang-Bang 제어 방식으로 거북이를 목표 방향으로 회전
4. 목표에 도달하면 자동으로 정지

토픽 구조:
- 구독(Subscribe): 
  * turtle1/pose (turtlesim/msg/Pose): 거북이의 현재 상태
  * goal_theta (std_msgs/msg/Float64): 목표 방향각 (라디안)
- 발행(Publish):
  * turtle1/cmd_vel (geometry_msgs/msg/Twist): 속도 명령
  * error (std_msgs/msg/Float64): 현재 제어 오차

파라미터:
- angular_speed (double): 회전 각속도 [rad/s] (기본값: 1.0)
- tolerance (double): 목표 도달 허용 오차 [rad] (기본값: 0.01)

사용법:
1. turtlesim_node 실행
2. 이 노드 실행
3. goal_theta 토픽으로 목표 방향 설정:
   ros2 topic pub /goal_theta std_msgs/msg/Float64 "data: 1.57"  # π/2 (90도)

작성자: [작성자명]
날짜: [작성일]
"""

# ROS2 기본 라이브러리들 import
import rclpy  # ROS2 Python 클라이언트 라이브러리 (Node 생성, 실행 등)
from rclpy.node import Node  # ROS2 노드의 기본 클래스
from turtlesim.msg import Pose  # turtlesim 패키지의 Pose 메시지 타입 (거북이 위치/방향 정보)
from geometry_msgs.msg import Twist  # 속도 명령을 위한 표준 메시지 타입 (선속도, 각속도)
from std_msgs.msg import Float64  # 64비트 실수 데이터를 위한 표준 메시지 타입
from rcl_interfaces.msg import SetParametersResult  # 파라미터 동적 변경 결과를 위한 메시지
import math  # 수학 연산을 위한 Python 표준 라이브러리

class TurtleConstantAngularController(Node):
    """
    거북이 회전 제어 클래스
    - 목표 각도(goal_theta)를 받아서 거북이를 해당 방향으로 회전시키는 컨트롤러
    - 일정한 각속도로 회전하여 목표 방향에 도달하면 정지
    - ROS2 파라미터를 통해 각속도와 허용 오차를 동적으로 조절 가능
    """
    def __init__(self):
        # 부모 클래스(Node) 초기화 - 노드 이름을 'turtle_constant_angular_controller'로 설정
        super().__init__('turtle_constant_angular_controller')
        
        # =============================================================================
        # ROS2 파라미터 설정 - 런타임에 변경 가능한 설정값들
        # =============================================================================
        # 파라미터 선언 (기본값과 함께)
        self.declare_parameter('angular_speed', 1.0)  # 회전 각속도 (rad/s)
        self.declare_parameter('tolerance', 0.01)     # 목표 도달 허용 오차 (rad)

        # 선언된 파라미터의 현재 값을 인스턴스 변수에 저장
        self.angular_speed = self.get_parameter('angular_speed').value
        self.tolerance = self.get_parameter('tolerance').value

        # 파라미터가 동적으로 변경될 때 호출될 콜백 함수 등록
        # 이를 통해 프로그램 실행 중에도 파라미터 값을 바꿀 수 있음
        self.add_on_set_parameters_callback(self.parameter_callback)

        # =============================================================================
        # 제어 변수 초기화
        # =============================================================================
        self.current_theta = 0.0  # 현재 거북이의 방향각 (라디안)
        self.target_theta = 0.0   # 목표 방향각 (goal_theta 토픽에서 받아옴)

        # =============================================================================
        # ROS2 토픽 구독자(Subscriber) 설정 - 다른 노드에서 발행하는 데이터를 받아옴
        # =============================================================================
        # 1. turtlesim의 거북이 위치/자세 정보 구독
        self.pose_subscriber = self.create_subscription(
            Pose,                # 메시지 타입: 위치(x,y)와 방향(theta) 정보 포함
            'turtle1/pose',      # 토픽 이름: turtlesim이 거북이 상태를 발행하는 토픽
            self.pose_callback,  # 콜백 함수: 메시지가 도착할 때마다 호출됨
            10                   # 큐 크기: 최대 10개 메시지까지 버퍼링
        )
        
        # 2. 목표 방향각 명령 구독 (외부에서 목표값 설정)
        self.goal_subscriber = self.create_subscription(
            Float64,            # 메시지 타입: 64비트 실수 (목표 각도 값)
            'goal_theta',       # 토픽 이름: 사용자 정의 토픽
            self.goal_callback, # 콜백 함수: 새로운 목표값이 올 때마다 호출
            10                  # 큐 크기
        )
        
        # =============================================================================
        # ROS2 토픽 발행자(Publisher) 설정 - 다른 노드로 데이터를 보냄
        # =============================================================================
        # 1. 거북이 속도 명령 발행 (turtlesim을 제어)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,              # 메시지 타입: 선속도(linear)와 각속도(angular) 포함
            'turtle1/cmd_vel',  # 토픽 이름: turtlesim이 속도 명령을 받는 토픽
            10                  # 큐 크기
        )
        
        # 2. 제어 오차 정보 발행 (모니터링/디버깅 용도)
        self.error_publisher = self.create_publisher(
            Float64,    # 메시지 타입: 계산된 오차 값
            'error',    # 토픽 이름: 사용자 정의 토픽
            10          # 큐 크기
        )

    def parameter_callback(self, params):
        """
        파라미터 동적 변경 콜백 함수
        - 실행 중에 파라미터가 변경되면 자동으로 호출됨
        - ros2 param set 명령어나 rqt_reconfigure 등으로 파라미터 변경 시 실행
        
        Args:
            params: 변경된 파라미터들의 리스트
        Returns:
            SetParametersResult: 파라미터 변경 성공/실패 결과
        """
        for param in params:
            if param.name == 'angular_speed':
                self.angular_speed = param.value
                self.get_logger().info(f"Updated angular_speed: {self.angular_speed}")
            elif param.name == 'tolerance':
                self.tolerance = param.value
                self.get_logger().info(f"Updated tolerance: {self.tolerance}")
        
        # 파라미터 변경이 성공했음을 알려주는 결과 객체 생성
        result = SetParametersResult()
        result.successful = True
        return result

    def pose_callback(self, msg):
        """
        거북이 자세 정보 수신 콜백 함수
        - turtlesim에서 발행하는 turtle1/pose 토픽을 받을 때마다 호출
        - 거북이의 현재 위치(x,y)와 방향(theta) 정보를 포함
        
        Args:
            msg (Pose): 거북이의 위치와 자세 정보
                       msg.x: x 좌표
                       msg.y: y 좌표  
                       msg.theta: 방향각 (라디안)
                       msg.linear_velocity: 선속도
                       msg.angular_velocity: 각속도
        """
        self.current_theta = msg.theta  # 현재 방향각 업데이트
        self.control()                  # 제어 함수 호출 (매번 새로운 자세 정보가 올 때마다 제어 수행)

    def goal_callback(self, msg):
        """
        목표 방향각 수신 콜백 함수
        - goal_theta 토픽으로부터 새로운 목표 방향을 받을 때마다 호출
        - 외부에서 거북이가 향해야 할 목표 방향을 설정할 때 사용
        
        Args:
            msg (Float64): 목표 방향각 (라디안 단위)
                          예: 0.0 = 동쪽, π/2 = 북쪽, π = 서쪽, 3π/2 = 남쪽
        """
        self.target_theta = msg.data
        self.get_logger().info(f"Received new goal_theta: {self.target_theta:.2f}")

    def control(self):
        """
        거북이 회전 제어 함수 (핵심 제어 로직)
        - 현재 방향과 목표 방향 사이의 오차를 계산
        - 오차에 따라 적절한 각속도 명령을 생성하여 거북이를 제어
        - Bang-Bang 제어 방식: 일정한 속도로 회전하다가 목표에 도달하면 정지
        """
        
        # =============================================================================
        # 1. 오차 계산 및 정규화
        # =============================================================================
        # 목표 방향과 현재 방향의 차이를 계산
        error = self.target_theta - self.current_theta
        
        # 각도 오차를 -π ~ π 범위로 정규화 (최단 경로로 회전하기 위함)
        # 예: 350° -> 10° 회전 시, +20° 대신 -340° 방향으로 계산되지 않도록 함
        # math.atan2(sin(θ), cos(θ))를 사용하면 자동으로 -π ~ π 범위로 정규화됨
        error = math.atan2(math.sin(error), math.cos(error))
        
        # =============================================================================
        # 2. 오차 정보 발행 (모니터링/디버깅 용도)
        # =============================================================================
        error_msg = Float64()
        error_msg.data = error
        self.error_publisher.publish(error_msg)

        # =============================================================================
        # 3. 속도 명령 생성 (Bang-Bang 제어)
        # =============================================================================
        twist_msg = Twist()
        
        # 오차가 허용 범위(tolerance)보다 크면 회전, 작으면 정지
        if abs(error) > self.tolerance:
            # 오차의 부호에 따라 회전 방향 결정
            # error > 0: 반시계방향 회전 (양의 각속도)
            # error < 0: 시계방향 회전 (음의 각속도)
            twist_msg.angular.z = self.angular_speed if error > 0 else -self.angular_speed
        else:
            # 목표에 충분히 가까우면 정지
            twist_msg.angular.z = 0.0
            
        # 선속도는 항상 0 (제자리 회전만 수행)
        twist_msg.linear.x = 0.0

        # =============================================================================
        # 4. 제어 명령 발행 및 로그 출력
        # =============================================================================
        self.cmd_vel_publisher.publish(twist_msg)
        
        # 현재 상태를 로그로 출력 (디버깅 및 모니터링)
        self.get_logger().info(
            f"Current theta: {self.current_theta:.2f}, "
            f"Goal theta: {self.target_theta:.2f}, "
            f"Error: {error:.2f}, "
            f"angular.z: {twist_msg.angular.z:.2f}"
        )

# =============================================================================
# 메인 함수 및 프로그램 진입점
# =============================================================================

def main(args=None):
    """
    메인 함수 - ROS2 노드를 실행하고 관리
    
    Args:
        args: 명령줄 인수 (일반적으로 None)
    
    실행 흐름:
    1. ROS2 초기화
    2. 노드 객체 생성
    3. 노드 실행 (무한 루프로 콜백 함수들이 계속 실행됨)
    4. 예외 처리 및 정리
    """
    # ROS2 시스템 초기화 (필수)
    rclpy.init(args=args)
    
    # 거북이 제어 노드 객체 생성
    node = TurtleConstantAngularController()
    
    try:
        # 노드 실행 시작 (무한 루프)
        # spin()은 노드가 토픽을 구독하고 콜백을 처리할 수 있도록 함
        # Ctrl+C를 누르거나 예외가 발생할 때까지 계속 실행됨
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        # Ctrl+C로 프로그램 종료 시 실행
        node.get_logger().info("Node interrupted by user.")
        
    finally:
        # 프로그램 종료 시 정리 작업
        node.destroy_node()  # 노드 객체 해제 (토픽, 서비스 등 정리)
        rclpy.shutdown()     # ROS2 시스템 종료

# Python 스크립트가 직접 실행될 때만 main() 함수 호출
# (다른 모듈에서 import될 때는 실행되지 않음)
if __name__ == '__main__':
    main()