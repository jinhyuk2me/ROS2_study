#!/usr/bin/env python3
"""
객체지향 State Machine 터틀 제어기 (Object-Oriented State Machine Turtle Controller)

이 파일은 클래식한 State Machine 디자인 패턴을 사용하여 터틀 제어를 구현한 
고급 소프트웨어 아키텍처 예제입니다. move_turtle.py와 동일한 기능을 제공하지만,
더욱 확장 가능하고 유지보수가 용이한 객체지향 방식으로 설계되었습니다.

핵심 디자인 패턴:
1. State Pattern (상태 패턴)
2. Template Method Pattern (템플릿 메서드 패턴)
3. Strategy Pattern (전략 패턴)

아키텍처 구조:
┌─────────────────────────────────────────────────────────────┐
│                  객체지향 State Machine                     │
├─────────────────────────────────────────────────────────────┤
│  StateResult (열거형)    │  ControllerState (추상 클래스)    │
│  - CONTINUE              │  - update() 추상 메서드          │
│  - COMPLETE              │                                  │
├─────────────────────────────────────────────────────────────┤
│  구체적 상태 클래스들 (ControllerState 상속)                │
│  ├─ RotateToGoalState    │  ├─ MoveToGoalState             │
│  ├─ RotateToFinalState   │  └─ GoalReachedState            │
├─────────────────────────────────────────────────────────────┤
│  StateTransitionManager  │  TurtleGoalController            │
│  - 상태 전환 규칙 관리    │  - 메인 제어기 노드              │
└─────────────────────────────────────────────────────────────┘

move_turtle.py와의 비교:
┌─────────────────────┬─────────────────────────────────────┐
│    move_turtle.py   │    move_turtle_stata_machine.py     │
├─────────────────────┼─────────────────────────────────────┤
│ 절차적 State Machine │ 객체지향 State Machine              │
│ if-elif 분기문      │ 다형성 활용                         │
│ 문자열 상태 관리     │ 객체 기반 상태 관리                 │
│ 단일 클래스 구조     │ 다중 클래스 분산 구조               │
│ 빠른 개발          │ 확장성 및 유지보수성                │
└─────────────────────┴─────────────────────────────────────┘

학습 목표:
- 고급 객체지향 디자인 패턴 이해
- State Machine의 다양한 구현 방법
- 소프트웨어 아키텍처 설계 원칙
- 코드의 확장성과 유지보수성 향상
- 추상화와 다형성의 실용적 활용

적용 분야:
- 복잡한 로봇 행동 제어
- 게임 AI 상태 관리
- 프로토콜 스택 구현
- 워크플로우 엔진

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
from turtlesim.msg import Pose                  # 터틀 위치/방향 정보
from geometry_msgs.msg import Twist             # 속도 명령
from std_msgs.msg import Float64, String        # 기본 데이터 타입
from rcl_interfaces.msg import SetParametersResult  # 파라미터 설정 결과

# 수학 및 제어 라이브러리
import math                                     # 수학 함수
from controller_tutorials.control_apps import PID  # 커스텀 PID 클래스

# =============================================================================
# 유틸리티 함수 섹션
# =============================================================================

def normalize_angle(angle):
    """
    각도 정규화 함수 (유틸리티)
    
    임의의 각도를 [-π, π] 범위로 정규화합니다.
    모든 상태 클래스에서 공통으로 사용하는 핵심 유틸리티 함수입니다.
    
    Args:
        angle (float): 정규화할 각도 (라디안)
        
    Returns:
        float: 정규화된 각도 (-π ≤ angle ≤ π)
    """
    return math.atan2(math.sin(angle), math.cos(angle))

# =============================================================================
# State Machine 핵심 구성 요소들
# =============================================================================

class StateResult:
    """
    상태 실행 결과 열거형 클래스 (Enumeration Class)
    
    각 상태의 update() 메서드가 반환할 수 있는 결과를 정의합니다.
    이는 상태 전환 로직을 명확하고 타입 안전하게 만듭니다.
    
    상수 정의:
    - CONTINUE: 현재 상태를 계속 유지 (목표 미달성)
    - COMPLETE: 현재 상태 완료 (다음 상태로 전환 필요)
    
    사용 이유:
    - 매직 넘버 방지 (0, 1 대신 의미있는 상수 사용)
    - 코드 가독성 향상
    - 실수 방지 (오타나 잘못된 값 사용 방지)
    """
    CONTINUE = 0    # 현재 상태 계속 실행
    COMPLETE = 1    # 현재 상태 완료, 다음 상태로 전환

class ControllerState:
    """
    추상 상태 클래스 (Abstract State Class)
    
    State Pattern의 핵심 구성 요소로, 모든 구체적 상태 클래스의 부모 클래스입니다.
    공통 인터페이스를 정의하고 템플릿 메서드 패턴을 구현합니다.
    
    디자인 패턴 적용:
    - State Pattern: 상태별 행동을 캡슐화
    - Template Method: 공통 구조 정의, 세부 구현은 서브클래스에 위임
    
    역할:
    1. 모든 상태 클래스의 공통 인터페이스 정의
    2. 제어기 객체에 대한 참조 보관
    3. 다형성을 통한 상태별 동작 실행
    
    서브클래스 구현 요구사항:
    - update() 메서드 필수 구현
    - (Twist, StateResult) 튜플 반환
    """
    
    def __init__(self, controller):
        """
        상태 클래스 초기화
        
        Args:
            controller: 메인 제어기 객체 참조 (의존성 주입)
        """
        self.controller = controller

    def update(self, current_pose):
        """
        상태별 제어 로직 실행 (추상 메서드)
        
        각 서브클래스에서 반드시 구현해야 하는 메서드입니다.
        현재 터틀 위치를 받아 속도 명령과 상태 결과를 반환합니다.
        
        Args:
            current_pose (Pose): 터틀의 현재 위치 및 방향
            
        Returns:
            tuple: (Twist 속도_명령, StateResult 상태_결과)
            
        Raises:
            NotImplementedError: 서브클래스에서 구현하지 않은 경우
        """
        raise NotImplementedError("update() must be implemented by subclasses")


# =============================================================================
# 구체적 상태 클래스들 (Concrete State Classes)
# =============================================================================

class RotateToGoalState(ControllerState):
    """
    1단계: 목표 방향으로 회전 상태 (Rotate to Goal Direction State)
    
    터틀을 목표 위치 방향으로 회전시키는 상태입니다.
    ControllerState를 상속받아 상태별 특화된 제어 로직을 구현합니다.
    
    State Pattern 적용:
    - 회전 제어 로직을 별도 클래스로 캡슐화
    - 다른 상태들과 완전히 독립적인 구현
    - 확장성: 새로운 회전 전략 쉽게 추가 가능
    
    제어 전략:
    - 각속도 제어만 사용 (선속도 = 0)
    - PID 기반 각도 오차 보정
    - 허용 오차 내 도달 시 다음 상태로 전환
    """
    
    def update(self, current_pose):
        """
        목표 방향 회전 제어 로직 실행
        
        Args:
            current_pose (Pose): 터틀의 현재 위치 및 방향
            
        Returns:
            tuple: (Twist 속도_명령, StateResult 상태_결과)
        """
        twist_msg = Twist()
        
        # 목표 방향 계산 (현재 위치에서 목표 위치로의 벡터 각도)
        desired_heading = math.atan2(
            self.controller.goal_pose.y - current_pose.y,
            self.controller.goal_pose.x - current_pose.x)
        
        # 각도 오차 계산 및 정규화
        error_angle = normalize_angle(desired_heading - current_pose.theta)
        
        # 각도 오차 정보 발행 (모니터링용)
        error_msg = Float64()
        error_msg.data = error_angle
        self.controller.angle_error_publisher.publish(error_msg)
        self.controller.state_publisher.publish(String(data="RotateToGoal"))
        self.controller.get_logger().info(f"[RotateToGoal] Heading error: {error_angle:.2f}")

        # 허용 오차 검사 및 제어 로직
        if abs(error_angle) > self.controller.angle_tolerance:
            # 목표 방향에 도달하지 못함 → PID 제어 계속
            angular_correction = self.controller.angular_pid.update(error_angle)
            twist_msg.angular.z = angular_correction
            twist_msg.linear.x = 0.0
            return twist_msg, StateResult.CONTINUE
        else:
            # 목표 방향 도달 완료 → 다음 상태로 전환
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
            self.controller.get_logger().info("Heading aligned. RotateToGoal complete.")
            return twist_msg, StateResult.COMPLETE

class MoveToGoalState(ControllerState):
    """
    2단계: 목표 위치로 이동 상태 (Move to Goal Position State)
    
    터틀을 목표 위치로 이동시키면서 동시에 방향을 보정하는 상태입니다.
    듀얼 PID 제어를 사용하여 효율적인 이동과 정확한 경로 유지를 달성합니다.
    
    State Pattern 장점:
    - 복잡한 듀얼 PID 로직을 독립적으로 관리
    - 이동 전략 변경 시 이 클래스만 수정하면 됨
    - 다른 상태들에 영향을 주지 않음
    
    제어 전략:
    - 듀얼 PID: 거리 제어(선속도) + 방향 제어(각속도)
    - 로봇 좌표계 기준 signed distance error 계산
    - 지속적인 헤딩 보정으로 직선 경로 유지
    """
    
    def update(self, current_pose):
        """
        목표 위치 이동 제어 로직 실행
        
        Args:
            current_pose (Pose): 터틀의 현재 위치 및 방향
            
        Returns:
            tuple: (Twist 속도_명령, StateResult 상태_결과)
        """
        twist_msg = Twist()
        
        # 위치 벡터 계산
        dx = self.controller.goal_pose.x - current_pose.x
        dy = self.controller.goal_pose.y - current_pose.y
        
        # 로봇 좌표계 기준 signed distance error (핵심 수학!)
        distance_error = dx * math.cos(current_pose.theta) + dy * math.sin(current_pose.theta)
        
        # 거리 오차 정보 발행
        error_msg = Float64()
        error_msg.data = distance_error
        self.controller.distance_error_publisher.publish(error_msg)
        self.controller.state_publisher.publish(String(data="MoveToGoal"))
        self.controller.get_logger().info(f"[MoveToGoal] Distance error: {distance_error:.2f}")

        # 거리 허용 오차 검사 및 듀얼 PID 제어
        if abs(distance_error) > self.controller.distance_tolerance:
            # 목표 위치에 도달하지 못함 → 듀얼 PID 제어 적용
            
            # 1. 선속도 제어 (거리 오차 기반)
            linear_correction = self.controller.linear_pid.update(distance_error)
            twist_msg.linear.x = linear_correction
            
            # 2. 각속도 제어 (방향 오차 기반) - 경로 보정
            desired_heading = math.atan2(
                self.controller.goal_pose.y - current_pose.y,
                self.controller.goal_pose.x - current_pose.x)
            angle_error = normalize_angle(desired_heading - current_pose.theta)
            angular_correction = self.controller.angular_pid.update(angle_error)
            twist_msg.angular.z = angular_correction
            
            return twist_msg, StateResult.CONTINUE
        else:
            # 목표 위치 도달 완료 → 다음 상태로 전환
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.controller.get_logger().info("Position reached. MoveToGoal complete.")
            return twist_msg, StateResult.COMPLETE

class RotateToFinalState(ControllerState):
    """
    3단계: 최종 방향으로 회전 상태 (Rotate to Final Orientation State)
    
    목표 위치에 도달한 후 최종 목표 방향으로 터틀을 회전시키는 상태입니다.
    정밀한 자세 제어를 통해 완전한 목표 달성을 보장합니다.
    
    객체지향 설계의 장점:
    - 최종 회전 로직을 독립적으로 관리
    - RotateToGoalState와 유사하지만 다른 목적 (목표 방향 vs 최종 방향)
    - 향후 다른 최종 자세 제어 전략으로 쉽게 교체 가능
    
    제어 전략:
    - 각속도 제어만 사용 (선속도 = 0)
    - 최종 목표 방향과의 오차 PID 보정
    - 완료 시 GoalReachedState로 전환
    """
    
    def update(self, current_pose):
        """
        최종 방향 회전 제어 로직 실행
        
        Args:
            current_pose (Pose): 터틀의 현재 위치 및 방향
            
        Returns:
            tuple: (Twist 속도_명령, StateResult 상태_결과)
        """
        twist_msg = Twist()
        
        # 최종 방향 오차 계산
        final_error = normalize_angle(self.controller.goal_pose.theta - current_pose.theta)
        
        # 최종 방향 오차 정보 발행
        error_msg = Float64()
        error_msg.data = final_error
        self.controller.angle_error_publisher.publish(error_msg)
        self.controller.state_publisher.publish(String(data="RotateToFinal"))
        self.controller.get_logger().info(f"[RotateToFinal] Orientation error: {final_error:.2f}")

        # 방향 허용 오차 검사 및 제어 로직
        if abs(final_error) > self.controller.angle_tolerance:
            # 최종 방향에 도달하지 못함 → PID 제어 계속
            angular_correction = self.controller.angular_pid.update(final_error)
            twist_msg.angular.z = angular_correction
            twist_msg.linear.x = 0.0
            return twist_msg, StateResult.CONTINUE
        else:
            # 최종 방향 달성 완료 → 목표 완전 달성!
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
            self.controller.get_logger().info("Final orientation reached. RotateToFinal complete.")
            return twist_msg, StateResult.COMPLETE

class GoalReachedState(ControllerState):
    """
    4단계: 목표 달성 상태 (Goal Reached Terminal State)
    
    모든 제어 목표를 달성한 후의 최종 상태입니다.
    State Pattern에서 흔히 볼 수 있는 "터미널 상태" 구현 예제입니다.
    
    터미널 상태의 특징:
    - 상태 전환이 더 이상 발생하지 않음 (자기 자신으로 전환)
    - 안정적인 정지 상태 유지
    - 새로운 목표 수신 시까지 대기
    
    객체지향 설계의 일관성:
    - 다른 상태들과 동일한 인터페이스 (update 메서드)
    - 다형성을 통한 균일한 처리
    - 상태 기반 로직의 명확한 종료점
    """
    
    def update(self, current_pose):
        """
        목표 달성 상태 유지 로직
        
        Args:
            current_pose (Pose): 터틀의 현재 위치 및 방향 (사용하지 않음)
            
        Returns:
            tuple: (Twist 정지_명령, StateResult.COMPLETE)
        """
        twist_msg = Twist()
        twist_msg.linear.x = 0.0    # 선속도 정지
        twist_msg.angular.z = 0.0   # 각속도 정지
        
        # 상태 정보 발행
        self.controller.state_publisher.publish(String(data="GoalReached"))
        self.controller.get_logger().info("Goal reached!")
        
        # 항상 COMPLETE를 반환하지만 StateTransitionManager에서 자기 자신으로 전환
        return twist_msg, StateResult.COMPLETE


# =============================================================================
# 상태 전환 관리자 (State Transition Manager)
# =============================================================================

class StateTransitionManager:
    """
    상태 전환 규칙 관리자 (State Transition Rules Manager)
    
    State Machine의 전환 로직을 중앙집중적으로 관리하는 클래스입니다.
    Strategy Pattern을 적용하여 상태 전환 규칙을 캡슐화했습니다.
    
    디자인 패턴 적용:
    - Strategy Pattern: 상태 전환 전략을 독립적으로 관리
    - Single Responsibility: 상태 전환 규칙만 담당
    - Open/Closed Principle: 새로운 상태 추가 시 이 클래스만 수정
    
    아키텍처 장점:
    1. 상태 전환 규칙의 중앙 집중 관리
    2. 각 상태 클래스는 자신의 제어 로직에만 집중
    3. 전환 규칙 변경 시 한 곳만 수정하면 됨
    4. 복잡한 전환 조건도 쉽게 추가 가능
    
    상태 전환 다이어그램:
    ┌─────────────────────────────────────────────────────────────┐
    │                    상태 전환 규칙                            │
    ├─────────────────────────────────────────────────────────────┤
    │  RotateToGoalState  ──COMPLETE──►  MoveToGoalState          │
    │                                         │                   │
    │  GoalReachedState   ◄──COMPLETE──  RotateToFinalState      │
    │        │                               ▲                   │
    │        └──COMPLETE──► GoalReachedState ─┘                   │
    │                                                             │
    │  모든 상태 ──CONTINUE──► 자기 자신 (상태 유지)                │
    └─────────────────────────────────────────────────────────────┘
    """
    
    def __init__(self, controller):
        """
        상태 전환 관리자 초기화
        
        Args:
            controller: 메인 제어기 객체 참조 (상태 인스턴스 생성용)
        """
        self.controller = controller

    def get_next_state(self, current_state, state_result):
        """
        현재 상태와 실행 결과를 바탕으로 다음 상태를 결정
        
        이 메서드는 State Machine의 핵심 로직을 구현합니다.
        상태 전환 규칙을 명확하고 확장 가능한 방식으로 정의합니다.
        
        전환 규칙:
        1. COMPLETE 결과 시 다음 단계 상태로 전환
        2. CONTINUE 결과 시 현재 상태 유지
        3. GoalReachedState는 터미널 상태 (자기 자신으로 전환)
        
        Args:
            current_state (ControllerState): 현재 실행 중인 상태 객체
            state_result (StateResult): 상태 실행 결과 (CONTINUE 또는 COMPLETE)
            
        Returns:
            ControllerState: 다음에 실행할 상태 객체
        """
        # 상태 완료 시 다음 단계로 전환
        if state_result == StateResult.COMPLETE:
            # isinstance를 사용한 타입 기반 전환 규칙
            # 향후 새로운 상태 추가 시 여기에 규칙 추가
            
            if isinstance(current_state, RotateToGoalState):
                # 1단계 완료 → 2단계: 목표 위치로 이동
                return MoveToGoalState(self.controller)
            elif isinstance(current_state, MoveToGoalState):
                # 2단계 완료 → 3단계: 최종 방향으로 회전
                return RotateToFinalState(self.controller)
            elif isinstance(current_state, RotateToFinalState):
                # 3단계 완료 → 4단계: 목표 달성 (터미널 상태)
                return GoalReachedState(self.controller)
            elif isinstance(current_state, GoalReachedState):
                # 터미널 상태: 자기 자신으로 전환 (상태 유지)
                return GoalReachedState(self.controller)
                
        # 상태 계속 실행 시 현재 상태 유지
        elif state_result == StateResult.CONTINUE:
            return current_state
            
        # 예외 상황 처리: 현재 상태 유지 (안전장치)
        return current_state


# =============================================================================
# 메인 제어기 노드 (Main Controller Node)
# =============================================================================

class TurtleGoalController(Node):
    """
    객체지향 State Machine 터틀 제어기 노드 (Main Controller with OOP State Machine)
    
    이 클래스는 State Machine 패턴을 활용한 터틀 제어의 중심 역할을 합니다.
    move_turtle.py와 동일한 기능을 제공하지만 더 나은 아키텍처를 가집니다.
    
    아키텍처 특징:
    1. Composition over Inheritance: 상태 객체들을 조합하여 동작
    2. Dependency Injection: 상태 객체들이 이 제어기를 참조
    3. Single Responsibility: 각 구성요소가 단일 책임을 가짐
    4. Open/Closed: 새로운 상태 추가 시 확장 용이
    
    역할 분담:
    - TurtleGoalController: ROS2 인터페이스 관리, 전체 조율
    - StateTransitionManager: 상태 전환 규칙 관리
    - 각 State 클래스: 상태별 제어 로직 담당
    - PID 클래스: 제어 알고리즘 담당
    
    객체 관계:
    ┌─────────────────────────────────────────────────────────────┐
    │                TurtleGoalController                         │
    │  ┌─────────────────┐    ┌─────────────────────────────────┐ │
    │  │ state_instance  │    │   state_transition_manager      │ │
    │  │ (current state) │◄──►│   (transition rules)            │ │
    │  └─────────────────┘    └─────────────────────────────────┘ │
    │  ┌─────────────────┐    ┌─────────────────────────────────┐ │
    │  │   angular_pid   │    │      linear_pid                 │ │
    │  │   (angle ctrl)  │    │      (distance ctrl)            │ │
    │  └─────────────────┘    └─────────────────────────────────┘ │
    └─────────────────────────────────────────────────────────────┘
    
    move_turtle.py와의 차이점:
    - 절차적 → 객체지향
    - if-elif → 다형성
    - 단일 클래스 → 다중 클래스 협력
    - 빠른 개발 → 장기 유지보수성
    """
    
    def __init__(self):
        """
        객체지향 State Machine 제어기 초기화
        
        초기화 과정:
        1. ROS2 파라미터 설정 (허용 오차, PID 게인)
        2. 듀얼 PID 제어기 생성 및 설정
        3. State Machine 구성요소 초기화
        4. ROS2 토픽 인터페이스 설정
        5. 동적 파라미터 콜백 등록
        """
        # 부모 클래스(Node) 초기화
        super().__init__('turtle_goal_controller')
        
        # =============================================================================
        # 1. ROS2 파라미터 선언 및 설정
        # =============================================================================
        
        # 제어 허용 오차 파라미터 (정밀도 제어)
        self.declare_parameter('angle_tolerance', 0.1)        # 각도 허용 오차 (라디안)
        self.declare_parameter('distance_tolerance', 0.1)     # 거리 허용 오차 (미터)

        # 각속도 PID 제어기 파라미터
        self.declare_parameter('angular_P', 2.0)              # 비례 게인
        self.declare_parameter('angular_I', 0.0)              # 적분 게인
        self.declare_parameter('angular_D', 0.0)              # 미분 게인
        self.declare_parameter('angular_max_state', 2.0)      # 최대 각속도
        self.declare_parameter('angular_min_state', -2.0)     # 최소 각속도

        # 선속도 PID 제어기 파라미터
        self.declare_parameter('linear_P', 1.0)               # 비례 게인
        self.declare_parameter('linear_I', 0.0)               # 적분 게인
        self.declare_parameter('linear_D', 0.0)               # 미분 게인
        self.declare_parameter('linear_max_state', 2.0)       # 최대 선속도
        self.declare_parameter('linear_min_state', -2.0)      # 최소 선속도

        # 허용 오차 파라미터 값 저장
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value

        # =============================================================================
        # 2. 듀얼 PID 제어기 초기화
        # =============================================================================
        
        # 각속도 PID 제어기 초기화 (방향 제어용)
        angular_P = self.get_parameter('angular_P').value
        angular_I = self.get_parameter('angular_I').value
        angular_D = self.get_parameter('angular_D').value
        angular_max_state = self.get_parameter('angular_max_state').value
        angular_min_state = self.get_parameter('angular_min_state').value
        
        self.angular_pid = PID()
        self.angular_pid.P = angular_P
        self.angular_pid.I = angular_I
        self.angular_pid.D = angular_D
        self.angular_pid.max_state = angular_max_state
        self.angular_pid.min_state = angular_min_state

        # 선속도 PID 제어기 초기화 (거리 제어용)
        linear_P = self.get_parameter('linear_P').value
        linear_I = self.get_parameter('linear_I').value
        linear_D = self.get_parameter('linear_D').value
        linear_max_state = self.get_parameter('linear_max_state').value
        linear_min_state = self.get_parameter('linear_min_state').value
        
        self.linear_pid = PID()
        self.linear_pid.P = linear_P
        self.linear_pid.I = linear_I
        self.linear_pid.D = linear_D
        self.linear_pid.max_state = linear_max_state
        self.linear_pid.min_state = linear_min_state

        # =============================================================================
        # 3. State Machine 구성요소 초기화
        # =============================================================================
        
        # 현재 실행 중인 상태 객체 (초기값 None - 목표 수신 시 설정)
        self.state_instance = None
        
        # 목표 위치 및 방향 정보 (goal_pose 토픽에서 수신)
        self.goal_pose = None
        
        # 상태 전환 관리자 생성 (전환 규칙 중앙 관리)
        # Dependency Injection: 이 제어기 객체를 주입
        self.state_transition_manager = StateTransitionManager(self)
        
        # =============================================================================
        # 4. ROS2 토픽 인터페이스 설정
        # =============================================================================
        
        # 구독자 (Subscriber) - 데이터 수신
        self.pose_subscriber = self.create_subscription(
            Pose, 'turtle1/pose', self.pose_callback, 10)
        self.goal_pose_subscriber = self.create_subscription(
            Pose, 'goal_pose', self.goal_pose_callback, 10)
            
        # 발행자 (Publisher) - 명령 및 상태 정보 송신
        self.cmd_vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.angle_error_publisher = self.create_publisher(Float64, 'angle_error', 10)
        self.distance_error_publisher = self.create_publisher(Float64, 'distance_error', 10)
        self.state_publisher = self.create_publisher(String, 'state', 10)
        
        # =============================================================================
        # 5. 동적 파라미터 콜백 등록
        # =============================================================================
        
        # 실시간 파라미터 변경을 위한 콜백 등록
        self.add_on_set_parameters_callback(self.parameter_callback)
    
    # =============================================================================
    # 콜백 메서드들 (Callback Methods)
    # =============================================================================
    
    def parameter_callback(self, params):
        """
        동적 파라미터 재구성 콜백 함수
        
        ROS2 파라미터 서버를 통한 실시간 제어 파라미터 변경을 처리합니다.
        객체지향 아키텍처에서도 move_turtle.py와 동일한 파라미터 인터페이스를 제공합니다.
        
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
        목표 위치 수신 콜백 함수 (State Machine 시작점)
        
        새로운 목표가 설정되면 State Machine을 초기화하고 첫 번째 상태를 시작합니다.
        객체지향 아키텍처의 핵심: 상태 객체 생성과 의존성 주입
        
        State Machine 시작 과정:
        1. 목표 위치 저장
        2. 첫 번째 상태 객체 생성 (RotateToGoalState)
        3. 상태 객체에 제어기 참조 주입 (Dependency Injection)
        
        Args:
            msg (Pose): 새로 설정된 목표 위치 및 방향
        """
        # 목표 위치 저장
        self.goal_pose = msg
        
        # State Machine 시작: 첫 번째 상태 객체 생성
        # Dependency Injection: 이 제어기 객체(self)를 상태에 주입
        self.state_instance = RotateToGoalState(self)
        
        self.get_logger().info(
            f"Received new goal pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}"
        )
    
    def pose_callback(self, msg):
        """
        터틀 위치 수신 콜백 함수 (State Machine 실행 엔진)
        
        이 메서드는 객체지향 State Machine의 실행 엔진 역할을 합니다.
        현재 상태 객체의 update() 메서드를 호출하고 상태 전환을 관리합니다.
        
        State Machine 실행 과정:
        1. 현재 상태 객체의 update() 호출 (다형성 활용)
        2. 상태 실행 결과 받기 (속도 명령 + 상태 결과)
        3. StateTransitionManager를 통해 다음 상태 결정
        4. 상태 전환 수행 (필요 시)
        5. 속도 명령 터틀에 전송
        
        객체지향의 핵심 장점:
        - 다형성: 어떤 상태든 동일한 인터페이스로 실행
        - 캡슐화: 각 상태의 로직이 독립적으로 실행
        - 확장성: 새로운 상태 추가가 용이
        
        Args:
            msg (Pose): 터틀의 현재 위치 및 방향
        """
        # 목표나 상태가 설정되지 않은 경우 대기
        if self.goal_pose is None or self.state_instance is None:
            return
            
        # 현재 상태 실행 (다형성 활용)
        # 어떤 상태 객체든 동일한 update() 인터페이스 사용
        twist_msg, status = self.state_instance.update(msg)
        
        # 상태 전환 로직 실행
        # StateTransitionManager에게 전환 결정 위임 (Single Responsibility)
        self.state_instance = self.state_transition_manager.get_next_state(
            self.state_instance, status)
        
        # 계산된 속도 명령 터틀에 전송
        self.cmd_vel_publisher.publish(twist_msg)

# =============================================================================
# 메인 함수 (프로그램 진입점)
# =============================================================================

def main(args=None):
    """
    객체지향 State Machine 터틀 제어기 메인 함수
    
    이 함수는 객체지향 아키텍처 기반 터틀 제어기의 생명주기를 관리합니다.
    move_turtle.py와 동일한 메인 함수 구조를 유지하여 일관성을 제공합니다.
    
    Args:
        args: 명령행 인수
    """
    # ROS2 시스템 초기화
    rclpy.init(args=args)
    
    # 객체지향 State Machine 제어기 생성
    node = TurtleGoalController()
    
    try:
        # 노드 실행 (State Machine 가동)
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 사용자 중단 처리
        node.get_logger().info("Node interrupted")
    finally:
        # 리소스 정리
        node.destroy_node()
        rclpy.shutdown()

# =============================================================================
# 스크립트 실행 진입점
# =============================================================================

if __name__ == '__main__':
    """
    객체지향 State Machine 터틀 제어기 직접 실행
    
    실행 방법:
    1. python3 move_turtle_stata_machine.py
    2. ros2 run controller_tutorials move_turtle_stata_machine
    
    아키텍처 비교:
    - move_turtle.py: 절차적 State Machine (빠른 개발)
    - move_turtle_stata_machine.py: 객체지향 State Machine (장기 유지보수)
    
    선택 기준:
    - 프로토타입/학습: move_turtle.py
    - 실제 제품/복잡한 시스템: move_turtle_stata_machine.py
    
    학습 포인트:
    1. 동일한 기능, 다른 아키텍처
    2. 디자인 패턴의 실용적 활용
    3. 확장성 vs 개발 속도의 트레이드오프
    4. 객체지향 설계 원칙의 적용
    """
    main()