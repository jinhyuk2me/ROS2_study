# ROS2 Launch 파일 - 여러 노드를 동시에 실행하는 스크립트
from launch import LaunchDescription          # Launch 시스템의 핵심 클래스
from launch_ros.actions import Node           # ROS2 노드를 실행하는 액션

def generate_launch_description():
    """
    Launch 파일의 메인 함수
    - ROS2가 자동으로 이 함수를 호출함
    - LaunchDescription 객체를 반환해야 함
    """
    return LaunchDescription([
        # 첫 번째 노드: turtlesim 시뮬레이터
        Node(
            namespace="turtlesim",           # 노드의 네임스페이스 (/turtlesim/)
            package='turtlesim',             # 사용할 패키지 (ROS2 기본 제공)
            executable='turtlesim_node',     # 실행할 노드 (거북이 창을 띄우는 노드)
            output='screen'                  # 로그 출력을 터미널에 표시
        ),
        
        # 두 번째 노드: 거북이 제어 퍼블리셔
        Node(
            namespace="pub_cmd_vel",         # 노드의 네임스페이스 (/pub_cmd_vel/)
            package="my_first_package",      # 우리가 만든 패키지
            executable="turtlesim_publisher", # 실행할 노드 (거북이를 원형으로 움직이는 퍼블리셔)
            output='screen',                 # 로그 출력을 터미널에 표시
        ),
    ])

# 사용법:
# ros2 launch my_first_package turtlesim_and_teleop.launch.py
# 
# 실행 결과:
# 1. 거북이 시뮬레이터 창이 열림
# 2. 거북이가 자동으로 원형으로 움직임
# 3. 두 노드가 동시에 실행됨