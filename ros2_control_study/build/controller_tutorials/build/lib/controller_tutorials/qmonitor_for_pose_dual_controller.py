#!/usr/bin/env python3
"""
거북이 듀얼 PID 제어 모니터링 GUI (Turtle Dual PID Controller Monitor)

이 파일은 pose_dual_controller.py의 제어 성능을 실시간으로 모니터링하고 시각화하는 
PyQt5 기반 GUI 애플리케이션입니다. ROS2와 GUI 프로그래밍의 융합 사례입니다.

주요 기능:
1. 실시간 터틀 위치 모니터링 (맵 시각화)
2. x, y 좌표 시간 그래프 (목표값 vs 현재값)
3. 마우스 클릭으로 목표 위치 설정
4. 가이드 라인 표시 (시작점 → 목표점)
5. 목표 도달 시 자동 가이드 라인 제거

기술 스택:
- ROS2: 토픽 통신, 노드 관리
- PyQt5: GUI 프레임워크
- matplotlib: 실시간 그래프 및 맵 시각화
- threading: ROS2와 GUI의 비동기 실행

GUI 구성:
┌─────────────────────────────────────────────────────────────┐
│                    Turtle Monitor GUI                       │
├─────────────────────┬───────────────────────────────────────┤
│                     │           X Coordinate Graph         │
│     Turtle MAP      │  (Current X vs Goal X over time)     │
│   (Click to set     ├───────────────────────────────────────┤
│    goal position)   │           Y Coordinate Graph         │
│                     │  (Current Y vs Goal Y over time)     │
└─────────────────────┴───────────────────────────────────────┘

학습 목표:
- ROS2와 GUI의 통합 방법
- 실시간 데이터 시각화 기법
- 멀티스레딩을 통한 비동기 프로그래밍
- 사용자 인터렉션 처리 (마우스 클릭)
- matplotlib과 PyQt5의 연동

사용법:
1. turtlesim_node 실행
2. pose_dual_controller.py 실행  
3. 이 모니터 실행
4. 맵에서 마우스 클릭으로 목표 설정
5. 실시간 제어 성능 관찰

작성자: [작성자명]
날짜: [작성일]
"""

# ROS2 관련 라이브러리
import rclpy  # ROS2 Python 클라이언트 라이브러리
from rclpy.node import Node  # ROS2 노드 기본 클래스
from turtlesim.msg import Pose  # 터틀 위치/방향 정보 메시지

# 시스템 및 스레딩 라이브러리
import threading  # ROS2 스핀을 별도 스레드에서 실행하기 위함
import math  # 수학 연산 (거리 계산, 삼각함수 등)
import sys  # 시스템 종료 처리

# PyQt5와 matplotlib 임포트 (GUI 및 시각화)
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout  # GUI 위젯들
from PyQt5.QtCore import QTimer  # 주기적 업데이트를 위한 타이머
from matplotlib.figure import Figure  # 그래프 생성
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas  # PyQt5-matplotlib 연동

# =============================================================================
# ROS2 모니터링 노드 클래스
# =============================================================================

class TurtleMonitor(Node):
    """
    터틀 모니터링 ROS2 노드 클래스
    
    이 클래스는 ROS2 노드로서 다음 역할을 수행합니다:
    1. 터틀의 현재 위치 데이터 수집 (turtle1/pose 구독)
    2. 목표 위치 데이터 수집 (goal_pose 구독)  
    3. GUI에서 설정된 목표 위치 발행 (goal_pose 발행)
    4. 가이드 라인 관리 (시작점 기록 및 표시)
    
    ROS2-GUI 통합의 핵심:
    - 이 노드가 ROS2와 PyQt5 GUI 사이의 데이터 브리지 역할
    - 별도 스레드에서 실행되어 GUI 메인 루프와 독립적으로 동작
    - GUI에서 이 노드의 데이터를 주기적으로 읽어서 시각화
    """
    
    def __init__(self):
        """
        터틀 모니터 노드 초기화
        
        초기화 과정:
        1. 데이터 저장 변수 초기화
        2. ROS2 토픽 구독자 설정 (데이터 수집용)
        3. ROS2 토픽 발행자 설정 (목표 설정용)
        """
        # 부모 클래스(Node) 초기화
        super().__init__('turtle_monitor')
        
        # =============================================================================
        # 데이터 저장 변수 초기화
        # =============================================================================
        self.turtle_pose = None      # 현재 터틀의 위치/방향 (turtle1/pose에서 수신)
        self.goal_pose = None        # 목표 위치/방향 (goal_pose에서 수신)
        self.guide_line_start = None # 가이드 선 시작점 (목표 설정 시점의 터틀 위치)

        # =============================================================================
        # ROS2 토픽 구독자 설정 (데이터 수집용)
        # =============================================================================
        # 1. 터틀의 현재 위치 구독
        self.create_subscription(
            Pose,                      # 메시지 타입: 위치(x,y)와 방향(theta) 정보
            'turtle1/pose',            # 토픽 이름: turtlesim이 발행하는 터틀 상태
            self.turtle_pose_callback, # 콜백 함수: 새 위치 데이터 수신 시 호출
            10                         # 큐 크기
        )
        
        # 2. 목표 위치 구독 (pose_dual_controller나 GUI에서 발행)
        self.create_subscription(
            Pose,                     # 메시지 타입: 목표 위치와 방향
            'goal_pose',              # 토픽 이름: 목표 설정 토픽
            self.goal_pose_callback,  # 콜백 함수: 새 목표 설정 시 호출
            10                        # 큐 크기
        )
        
        # =============================================================================
        # ROS2 토픽 발행자 설정 (목표 설정용)
        # =============================================================================
        # GUI에서 마우스 클릭으로 설정된 목표를 pose_dual_controller로 전달
        self.goal_pub = self.create_publisher(
            Pose,        # 메시지 타입: 목표 위치와 방향
            'goal_pose', # 토픽 이름: pose_dual_controller가 구독하는 토픽
            10           # 큐 크기
        )

    def turtle_pose_callback(self, msg):
        """
        터틀 위치 데이터 수신 콜백 함수
        
        turtlesim에서 발행하는 터틀의 현재 위치를 받아서 저장합니다.
        GUI에서 이 데이터를 읽어서 실시간 맵과 그래프를 업데이트합니다.
        
        Args:
            msg (Pose): 터틀의 현재 위치/방향 정보
                       - msg.x, msg.y: 현재 위치 좌표
                       - msg.theta: 현재 방향각
        """
        # 최신 터틀 위치 저장 (GUI에서 주기적으로 읽어감)
        self.turtle_pose = msg

    def goal_pose_callback(self, msg):
        """
        목표 위치 데이터 수신 콜백 함수
        
        새로운 목표가 설정될 때마다 호출되며, 다음 작업을 수행합니다:
        1. 목표 위치 저장 (GUI 시각화용)
        2. 가이드 라인 시작점 기록 (현재 터틀 위치)
        3. 로그 출력 (디버깅용)
        
        가이드 라인의 의미:
        - 시작점: 목표가 설정된 시점의 터틀 위치
        - 끝점: 설정된 목표 위치
        - 용도: 터틀이 목표를 향해 올바르게 이동하는지 시각적 확인
        
        Args:
            msg (Pose): 새로 설정된 목표 위치/방향 정보
        """
        # 새로운 목표 위치 저장
        self.goal_pose = msg
        
        # 가이드 라인을 위한 시작점 기록
        # 목표가 설정된 시점의 터틀 위치를 기록하여 나중에 선으로 연결
        if self.turtle_pose is not None:
            self.guide_line_start = (self.turtle_pose.x, self.turtle_pose.y)
            self.get_logger().info(
                f"Guide line set from ({self.turtle_pose.x:.2f}, {self.turtle_pose.y:.2f}) to ({msg.x:.2f}, {msg.y:.2f})"
            )

# =============================================================================
# ROS2 스레딩 함수
# =============================================================================

def ros_spin(node):
    """
    ROS2 노드를 별도 스레드에서 실행하기 위한 함수
    
    ROS2-GUI 통합에서 중요한 개념:
    - ROS2 노드와 PyQt5 GUI는 각각 자신만의 이벤트 루프를 가짐
    - ROS2: rclpy.spin() - 토픽/서비스 처리
    - PyQt5: app.exec_() - GUI 이벤트 처리
    - 두 루프를 동시에 실행하려면 별도 스레드가 필요
    
    멀티스레딩 구조:
    ┌─────────────────┐    ┌─────────────────┐
    │  Main Thread    │    │  ROS2 Thread   │
    │                 │    │                 │
    │  PyQt5 GUI      │    │  ROS2 Node      │
    │  - 시각화       │◄──►│  - 토픽 통신    │
    │  - 사용자 입력  │    │  - 데이터 수집  │
    │  - 타이머       │    │  - 콜백 처리    │
    └─────────────────┘    └─────────────────┘
    
    Args:
        node: 실행할 ROS2 노드 객체
    """
    # ROS2 노드를 무한 루프로 실행 (토픽 콜백 처리)
    # 이 함수는 별도 스레드에서 실행되어 GUI를 블록하지 않음
    rclpy.spin(node)

# =============================================================================
# PyQt5 메인 윈도우 클래스 (GUI 구현)
# =============================================================================

class MainWindow(QMainWindow):
    """
    터틀 모니터링 GUI 메인 윈도우 클래스
    
    이 클래스는 PyQt5 기반 GUI의 핵심으로 다음을 구현합니다:
    1. 3개 영역 레이아웃 (맵 + x그래프 + y그래프)
    2. 실시간 데이터 시각화 (100ms 간격 업데이트)
    3. 마우스 인터렉션 (클릭으로 목표 설정)
    4. matplotlib-PyQt5 통합
    
    GUI 구조:
    ┌─────────────────────────────────────────────────────────────┐
    │                      Main Window                            │
    │  ┌─────────────────┐  ┌─────────────────────────────────┐  │
    │  │                 │  │        X Coordinate Graph       │  │
    │  │   Turtle MAP    │  │  ┌─────────────────────────────┐ │  │
    │  │                 │  │  │ Current X vs Goal X (time) │ │  │
    │  │ (Click to set   │  │  └─────────────────────────────┘ │  │
    │  │  goal position) │  ├─────────────────────────────────┤  │
    │  │                 │  │        Y Coordinate Graph       │  │
    │  │                 │  │  ┌─────────────────────────────┐ │  │
    │  │                 │  │  │ Current Y vs Goal Y (time) │ │  │
    │  │                 │  │  └─────────────────────────────┘ │  │
    │  └─────────────────┘  └─────────────────────────────────┘  │
    └─────────────────────────────────────────────────────────────┘
    
    기술적 특징:
    - 실시간 업데이트: QTimer를 사용한 100ms 주기 갱신
    - 이벤트 기반: matplotlib 이벤트를 ROS2 토픽 발행과 연결
    - 데이터 히스토리: 시간에 따른 좌표 변화 추적
    - 레이아웃 관리: stretch factor를 활용한 크기 조절
    """
    
    def __init__(self, node):
        """
        GUI 메인 윈도우 초기화
        
        초기화 과정:
        1. 데이터 히스토리 변수 초기화
        2. GUI 레이아웃 구성 (수평 분할 → 왼쪽 맵, 오른쪽 그래프)
        3. matplotlib 캔버스 생성 및 배치
        4. 이벤트 연결 (마우스 클릭)
        5. 실시간 업데이트 타이머 설정
        
        Args:
            node (TurtleMonitor): ROS2 노드 객체 (데이터 소스)
        """
        # 부모 클래스(QMainWindow) 초기화
        super().__init__()
        
        # ROS2 노드 참조 저장 (데이터 읽기 및 토픽 발행용)
        self.node = node
        self.setWindowTitle("Turtle Monitor with Coordinate Graphs")

        # =============================================================================
        # 데이터 히스토리 변수 초기화 (시간에 따른 좌표 변화 추적용)
        # =============================================================================
        self.time_counter = 0.0        # 시간 카운터 (0.1초씩 증가)
        self.time_history = []         # 시간 히스토리 (x축 데이터)
        self.x_history = []            # 터틀 x 좌표 히스토리
        self.goal_x_history = []       # 목표 x 좌표 히스토리
        self.y_history = []            # 터틀 y 좌표 히스토리
        self.goal_y_history = []       # 목표 y 좌표 히스토리

        # =============================================================================
        # GUI 메인 레이아웃 구성 (수평 분할)
        # =============================================================================
        # 중앙 위젯 및 메인 레이아웃 생성
        main_widget = QWidget()
        main_layout = QHBoxLayout()  # 수평 배치: 왼쪽 맵, 오른쪽 그래프

        # =============================================================================
        # 왼쪽 영역: 터틀 맵 (matplotlib 캔버스)
        # =============================================================================
        # matplotlib Figure 및 Canvas 생성
        self.figure_map = Figure()                      # 맵 그리기용 Figure
        self.canvas = FigureCanvas(self.figure_map)     # PyQt5-matplotlib 연동 Canvas
        self.ax = self.figure_map.add_subplot(111)      # 서브플롯 (맵 그리기 영역)
        main_layout.addWidget(self.canvas, stretch=1)   # 레이아웃에 추가 (비율 1)

        # 마우스 클릭 이벤트 연결 (클릭 좌표를 목표로 설정)
        # matplotlib의 이벤트 시스템을 사용하여 마우스 클릭을 감지
        self.canvas.mpl_connect('button_press_event', self.on_map_click)

        # =============================================================================
        # 오른쪽 영역: x, y 좌표 그래프 (수직 배치)
        # =============================================================================
        # 그래프 컨테이너 위젯 및 레이아웃 생성
        graph_widget = QWidget()
        graph_layout = QVBoxLayout()  # 수직 배치: 위 x그래프, 아래 y그래프

        # x 좌표 그래프 캔버스 설정
        self.figure_x = Figure(figsize=(4, 2))              # 그래프 크기 설정
        self.figure_x.subplots_adjust(right=0.75)           # 범례 공간 확보
        self.canvas_x = FigureCanvas(self.figure_x)         # Canvas 생성
        self.ax_x = self.figure_x.add_subplot(111)          # 서브플롯 생성
        graph_layout.addWidget(self.canvas_x)               # 레이아웃에 추가

        # y 좌표 그래프 캔버스 설정 (x와 동일한 구조)
        self.figure_y = Figure(figsize=(4, 2))
        self.figure_y.subplots_adjust(right=0.75)
        self.canvas_y = FigureCanvas(self.figure_y)
        self.ax_y = self.figure_y.add_subplot(111)
        graph_layout.addWidget(self.canvas_y)

        # 그래프 위젯 설정 완료
        graph_widget.setLayout(graph_layout)
        main_layout.addWidget(graph_widget, stretch=2)  # 비율 2 (맵보다 크게)

        # 메인 레이아웃 설정 완료
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        # =============================================================================
        # 실시간 업데이트 타이머 설정
        # =============================================================================
        # QTimer를 사용하여 100ms(10Hz) 간격으로 GUI 업데이트
        # 이는 ROS2 토픽 주기(보통 50-100Hz)보다 낮은 주기로 설정하여 GUI 성능 최적화
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_all)  # 타이머 만료 시 update_all 호출
        self.timer.start(100)                        # 100ms 간격으로 시작

    def on_map_click(self, event):
        """
        맵 클릭 이벤트 핸들러 (사용자 인터렉션)
        
        matplotlib 캔버스에서 마우스 클릭 시 자동으로 호출되는 콜백 함수입니다.
        클릭한 좌표를 새로운 목표 위치로 설정하여 pose_dual_controller에 전달합니다.
        
        이벤트 처리 흐름:
        1. 클릭 위치가 맵 영역인지 확인
        2. 클릭 좌표 추출 (matplotlib 좌표계)
        3. Pose 메시지 생성 (x, y, theta=0)
        4. ROS2 토픽으로 발행
        
        Args:
            event: matplotlib 마우스 이벤트 객체
                  - event.inaxes: 클릭된 축 (맵 영역 확인용)
                  - event.xdata, event.ydata: 클릭 좌표
        """
        # 클릭 이벤트가 맵 영역(self.ax)에서 발생했는지 확인
        # 다른 영역(그래프 등) 클릭은 무시
        if event.inaxes == self.ax:
            # matplotlib 좌표계에서 클릭 위치 추출
            x_click = event.xdata  # 맵에서의 x 좌표
            y_click = event.ydata  # 맵에서의 y 좌표

            # ROS2 Pose 메시지 생성
            goal_msg = Pose()
            goal_msg.x = x_click      # 목표 x 좌표
            goal_msg.y = y_click      # 목표 y 좌표  
            goal_msg.theta = 0.0      # 목표 방향 (클릭에서는 방향 지정 안함)

            # pose_dual_controller로 목표 위치 전송
            # 이 토픽을 듀얼 PID 제어기가 구독하여 터틀을 해당 위치로 이동시킴
            self.node.goal_pub.publish(goal_msg)
            self.node.get_logger().info(f"Published new goal: ({x_click:.2f}, {y_click:.2f})")

    def update_all(self):
        """
        실시간 GUI 업데이트 메인 함수 (핵심 시각화 로직)
        
        이 함수는 QTimer에 의해 100ms마다 호출되어 다음 작업을 수행합니다:
        1. 시간 및 좌표 데이터 히스토리 업데이트
        2. 터틀 맵 시각화 (위치, 방향, 목표, 가이드라인)
        3. x, y 좌표 시간 그래프 업데이트
        4. 목표 도달 시 가이드라인 제거
        
        실시간 시각화의 핵심:
        - ROS2 데이터 → GUI 데이터 변환
        - matplotlib을 통한 동적 그래프 생성
        - 사용자 친화적 인터페이스 제공
        """
        
        # =============================================================================
        # 1. 시간 및 데이터 히스토리 업데이트
        # =============================================================================
        # 시간 카운터 증가 (100ms = 0.1초 간격)
        self.time_counter += 0.1
        self.time_history.append(self.time_counter)

        # 터틀의 현재 좌표 업데이트 (ROS2 노드에서 데이터 읽기)
        if self.node.turtle_pose is not None:
            # 최신 터틀 위치 사용
            current_x = self.node.turtle_pose.x
            current_y = self.node.turtle_pose.y
        else:
            # 데이터가 없으면 이전 값 유지 (초기값 0)
            current_x = self.x_history[-1] if self.x_history else 0
            current_y = self.y_history[-1] if self.y_history else 0
        self.x_history.append(current_x)
        self.y_history.append(current_y)

        # 목표 좌표 업데이트 (목표가 설정되지 않으면 이전 값 유지)
        if self.node.goal_pose is not None:
            # 최신 목표 위치 사용
            goal_x = self.node.goal_pose.x
            goal_y = self.node.goal_pose.y
        else:
            # 목표가 없으면 이전 목표 유지 (초기값 0)
            goal_x = self.goal_x_history[-1] if self.goal_x_history else 0
            goal_y = self.goal_y_history[-1] if self.goal_y_history else 0
        self.goal_x_history.append(goal_x)
        self.goal_y_history.append(goal_y)

        # =============================================================================
        # 2. 터틀 맵 시각화 업데이트
        # =============================================================================
        # 맵 축 초기화 (이전 그리기 내용 제거)
        self.ax.clear()
        
        # 맵 범위 설정 (turtlesim 환경과 동일: 0~11)
        self.ax.set_xlim(0, 11)
        self.ax.set_ylim(0, 11)
        self.ax.set_aspect('equal')  # 가로세로 비율 1:1 유지
        self.ax.grid(True)           # 격자 표시
        self.ax.set_title("Turtle MAP")

        # 현재 터틀 위치 및 방향 표시 (파란색 화살표와 점)
        if self.node.turtle_pose is not None:
            x = self.node.turtle_pose.x      # 터틀 x 좌표
            y = self.node.turtle_pose.y      # 터틀 y 좌표
            theta = self.node.turtle_pose.theta  # 터틀 방향각
            
            # 방향 화살표 생성 (터틀이 바라보는 방향 표시)
            arrow_length = 0.5  # 화살표 길이
            dx = arrow_length * math.cos(theta)  # x 방향 성분
            dy = arrow_length * math.sin(theta)  # y 방향 성분
            self.ax.arrow(x, y, dx, dy, head_width=0.3, head_length=0.3, fc='blue', ec='blue')
            
            # 터틀 위치 점 표시
            self.ax.plot(x, y, 'bo')  # 파란 원점
            
        # 목표 위치 표시 (빨간 점)
        if self.node.goal_pose is not None:
            gx = self.node.goal_pose.x
            gy = self.node.goal_pose.y
            self.ax.plot(gx, gy, 'ro', markersize=8)  # 빨간 원점 (크기 8)
            
        # 가이드 라인 표시 (목표 설정 시점 → 목표 위치)
        # 터틀이 어느 경로로 이동해야 하는지 시각적으로 확인 가능
        if self.node.guide_line_start is not None and self.node.goal_pose is not None:
            start_x, start_y = self.node.guide_line_start  # 가이드라인 시작점
            goal_x = self.node.goal_pose.x                 # 가이드라인 끝점 (목표)
            goal_y = self.node.goal_pose.y
            self.ax.plot([start_x, goal_x], [start_y, goal_y], 'r--')  # 빨간 점선
            
        # 목표 도달 검사 및 가이드 라인 자동 제거
        # 터틀이 목표에 충분히 가까워지면 가이드라인을 제거하여 화면 정리
        if self.node.turtle_pose is not None and self.node.goal_pose is not None:
            # 유클리드 거리 계산
            dist = math.sqrt((self.node.turtle_pose.x - self.node.goal_pose.x) ** 2 +
                             (self.node.turtle_pose.y - self.node.goal_pose.y) ** 2)
            # 거리 임계값 (0.1) 이하이면 목표 도달로 간주
            if dist < 0.1:
                self.node.guide_line_start = None  # 가이드라인 제거

        # 맵 캔버스 다시 그리기 (변경사항 화면에 반영)
        self.canvas.draw()

        # =============================================================================
        # 3. X 좌표 시간 그래프 업데이트 (제어 성능 분석용)
        # =============================================================================
        # x 그래프 축 초기화
        self.ax_x.clear()
        
        # 현재 x 좌표와 목표 x 좌표를 시간에 따라 플롯
        self.ax_x.plot(self.time_history, self.x_history, label="Current X", color='blue')      # 실제 위치
        self.ax_x.plot(self.time_history, self.goal_x_history, label="Goal X", color='red',     # 목표 위치
                       linestyle='--')  # 점선으로 구분
        
        # 그래프 설정
        self.ax_x.set_title("X Coordinate")
        self.ax_x.set_xlabel("Time (s)")
        self.ax_x.set_ylabel("X Value")
        self.ax_x.grid(True)
        
        # 시간 축 범위 설정 (최근 10초간 데이터만 표시 - 슬라이딩 윈도우)
        if self.time_counter < 10:
            self.ax_x.set_xlim(0, 10)                                        # 초기 10초는 0~10초 고정
        else:
            self.ax_x.set_xlim(self.time_counter - 10, self.time_counter)    # 10초 이후는 슬라이딩
            
        # 범례 설정 (그래프 오른쪽 바깥에 표시)
        self.ax_x.legend(loc='upper left', bbox_to_anchor=(1, 1))
        
        # x 그래프 캔버스 다시 그리기
        self.canvas_x.draw()

        # =============================================================================
        # 4. Y 좌표 시간 그래프 업데이트 (제어 성능 분석용)
        # =============================================================================
        # y 그래프 축 초기화
        self.ax_y.clear()
        
        # 현재 y 좌표와 목표 y 좌표를 시간에 따라 플롯
        self.ax_y.plot(self.time_history, self.y_history, label="Current Y", color='blue')      # 실제 위치
        self.ax_y.plot(self.time_history, self.goal_y_history, label="Goal Y", color='red',     # 목표 위치
                       linestyle='--')  # 점선으로 구분
        
        # 그래프 설정 (x 그래프와 동일)
        self.ax_y.set_title("Y Coordinate")
        self.ax_y.set_xlabel("Time (s)")
        self.ax_y.set_ylabel("Y Value")
        self.ax_y.grid(True)
        
        # 시간 축 범위 설정 (x 그래프와 동일한 슬라이딩 윈도우)
        if self.time_counter < 10:
            self.ax_y.set_xlim(0, 10)
        else:
            self.ax_y.set_xlim(self.time_counter - 10, self.time_counter)
            
        # 범례 설정
        self.ax_y.legend(loc='upper left', bbox_to_anchor=(1, 1))
        
        # y 그래프 캔버스 다시 그리기
        self.canvas_y.draw()

# =============================================================================
# 메인 함수 (프로그램 진입점 및 ROS2-GUI 통합 관리)
# =============================================================================

def main(args=None):
    """
    터틀 모니터링 애플리케이션 메인 함수
    
    이 함수는 전체 애플리케이션의 생명주기를 관리하며 다음을 수행합니다:
    1. ROS2 시스템 초기화
    2. 모니터링 노드 생성
    3. 멀티스레드 설정 (ROS2와 GUI 분리)
    4. PyQt5 GUI 실행
    5. 종료 시 정리 작업
    
    실행 흐름:
    ┌─────────────────────────────────────────────────────────────┐
    │                      프로그램 시작                           │
    ├─────────────────────────────────────────────────────────────┤
    │  1. ROS2 초기화 및 노드 생성                                │
    │  2. ROS2 스레드 시작 (백그라운드)                           │
    │  3. PyQt5 GUI 시작 (메인 스레드)                            │
    │     ├─ 사용자 인터렉션 처리                                 │
    │     ├─ 실시간 시각화 수행                                   │
    │     └─ ROS2 데이터 읽기                                     │
    │  4. GUI 종료 시 정리 작업                                   │
    │  5. 프로그램 종료                                           │
    └─────────────────────────────────────────────────────────────┘
    
    Args:
        args: 명령행 인수 (일반적으로 None)
    """
    
    # =============================================================================
    # 1. ROS2 시스템 초기화 및 노드 생성
    # =============================================================================
    # ROS2 클라이언트 라이브러리 초기화
    rclpy.init(args=args)
    
    # 터틀 모니터링 노드 생성 (토픽 구독/발행 설정 포함)
    node = TurtleMonitor()

    # =============================================================================
    # 2. 멀티스레드 설정 (ROS2와 GUI 분리 실행)
    # =============================================================================
    # ROS2 노드를 별도 스레드에서 실행 (GUI 블록킹 방지)
    # daemon=True: 메인 프로그램 종료 시 자동으로 스레드도 종료
    spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    # =============================================================================
    # 3. PyQt5 GUI 애플리케이션 실행 (메인 스레드)
    # =============================================================================
    # PyQt5 애플리케이션 객체 생성
    app = QApplication(sys.argv)
    
    # 메인 윈도우 생성 및 ROS2 노드 연결
    window = MainWindow(node)
    window.show()  # 윈도우 화면에 표시
    
    # GUI 이벤트 루프 시작 (사용자 종료 시까지 블록킹)
    ret = app.exec_()  # PyQt5 전용 (PyQt6는 app.exec())

    # =============================================================================
    # 4. 종료 시 정리 작업
    # =============================================================================
    # ROS2 노드 정리 (토픽 구독/발행 해제)
    node.destroy_node()
    
    # ROS2 시스템 종료
    rclpy.shutdown()
    
    # 프로그램 종료 (GUI 애플리케이션의 리턴 코드 전달)
    sys.exit(ret)

# =============================================================================
# 스크립트 실행 진입점
# =============================================================================

if __name__ == '__main__':
    """
    스크립트가 직접 실행될 때 main 함수 호출
    
    이 블록은 다음 경우에 실행됩니다:
    1. python3 qmonitor_for_pose_dual_controller.py
    2. ros2 run controller_tutorials qmonitor_for_pose_dual_controller
    
    다른 모듈에서 import할 때는 실행되지 않습니다.
    """
    main()