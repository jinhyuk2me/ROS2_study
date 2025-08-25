# 필요한 라이브러리들을 import
import time  # 시간 관련 함수 사용
import rclpy as rp  # ROS2 Python 클라이언트 라이브러리
from rclpy.action import ActionServer  # 액션 서버 기능
from rclpy.executors import MultiThreadedExecutor  # 멀티스레드 실행기
from rclpy.node import Node  # ROS2 노드 기본 클래스

# 메시지 타입들
from turtlesim.msg import Pose  # 거북이 위치 정보
from geometry_msgs.msg import Twist  # 속도 명령
from my_first_package_msgs.action import DistTurtle  # 자체 정의한 액션
from my_first_package.my_subscriber import TurtlesimSubscriber  # 구독자 클래스

from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange

import math  # 수학 계산 함수

# 거북이 위치를 구독하고 액션 서버에 전달하는 클래스
class TurtleSub_Action(TurtlesimSubscriber):
    def __init__(self, ac_server):
        super().__init__()  # 부모 클래스(TurtlesimSubscriber) 초기화
        self.ac_server = ac_server  # 액션 서버 참조 저장

    def callback(self, msg):
        """거북이 위치가 업데이트될 때마다 호출되는 함수"""
        self.ac_server.current_pose = msg  # 받은 위치 정보를 액션 서버에 전달

# 거북이를 지정된 거리만큼 이동시키는 액션 서버
class DistTurtleServer(Node):
    def __init__(self):
        super().__init__('dist_turtle_action_server')  # 노드 이름 설정
        
        # 클래스 변수들 초기화
        self.total_dist = 0  # 현재까지 이동한 총 거리
        self.is_first_time = True  # 첫 번째 계산인지 확인하는 플래그
        self.current_pose = Pose()  # 현재 거북이 위치
        self.previous_pose = Pose()  # 이전 거북이 위치
        
        # 거북이 속도 명령을 보내는 퍼블리셔 생성
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # 액션 서버 생성 (DistTurtle 액션, 'dist_turtle' 이름, excute_callback 함수)
        self.action_server = ActionServer(self, DistTurtle, 'dist_turtle', self.excute_callback)

        self.get_logger().info("DistTurtleServer is created")

        # 파라미터 설명자 생성 (quantile_time용)
        # - 파라미터의 유효 범위와 설명을 정의
        param_desc_quantile = ParameterDescriptor(
            description = "quantile_time description",
            floating_point_range = [FloatingPointRange(
                from_value = 0.0,    # 최소값: 0%
                to_value = 1.0,      # 최대값: 100% 
                step = 0.01          # 변경 단위: 1%씩
            )]
        )

        # 파라미터 선언 및 기본값 설정
        self.declare_parameter('quantile_time', 0.75, param_desc_quantile)  # 75% 지점에서 알림
        self.declare_parameter('almost_goal_time', 0.95)                    # 95% 지점 (거의 도착)

        # 선언한 파라미터들의 현재 값을 가져오기
        (quantile_time, almost_goal_time) = self.get_parameters(['quantile_time', 'almost_goal_time'])
        
        # 파라미터 값을 클래스 변수에 저장 (실제 사용을 위해)
        self.quantile_time = quantile_time.value      # 알림 지점 (기본: 0.75 = 75%)
        self.almost_goal_time = almost_goal_time.value # 거의 도착 지점 (기본: 0.95 = 95%)

        # 설정된 파라미터 값들을 로그로 출력
        output_msg = "quantile_time is " + str(self.quantile_time)
        output_msg = output_msg + " and almost_goal_time is " + str(self.almost_goal_time) + "."
        self.get_logger().info(output_msg)

        # 파라미터가 실시간으로 변경될 때 호출될 콜백 함수 등록
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """파라미터가 실시간으로 변경될 때 호출되는 콜백 함수"""
        # 변경된 모든 파라미터들을 순회하며 처리
        for param in params:
            print(param.name, "is changed to ", param.value)

            # quantile_time 파라미터가 변경된 경우
            if param.name == 'quantile_time':
                self.quantile_time = param.value  # 새 값으로 업데이트
            # almost_goal_time 파라미터가 변경된 경우
            elif param.name == 'almost_goal_time':
                self.almost_goal_time = param.value  # 새 값으로 업데이트

        # 업데이트된 파라미터 값들을 확인용으로 출력
        print(f"quantile_time: {self.quantile_time}, almost_goal_time: {self.almost_goal_time}")

        # 파라미터 변경이 성공했음을 ROS2에게 알림
        return SetParametersResult(successful=True)

    def calc_diff_pose(self):
        """현재 위치와 이전 위치 사이의 거리를 계산하는 함수"""
        if self.is_first_time:
            # 첫 번째 호출일 때는 이전 위치를 현재 위치로 설정
            self.previous_pose.x = self.current_pose.x
            self.previous_pose.y = self.current_pose.y
            self.is_first_time = False

        # 두 점 사이의 유클리드 거리 계산 (피타고라스 정리)
        diff_dist = math.sqrt((self.current_pose.x - self.previous_pose.x)**2 + 
                             (self.current_pose.y - self.previous_pose.y)**2)

        # 현재 위치를 다음 계산을 위한 이전 위치로 저장
        self.previous_pose = self.current_pose

        return diff_dist  # 계산된 거리 반환

    def excute_callback(self, goal_handle):
        """액션이 실행될 때 호출되는 메인 콜백 함수"""
        # 피드백 메시지 객체 생성 (진행 상황을 클라이언트에게 전달)
        feedback_msg = DistTurtle.Feedback()

        # Twist 메시지 생성 및 목표값 설정
        msg = Twist()
        msg.linear.x = goal_handle.request.linear_x  # 요청받은 선형 속도
        msg.angular.z = goal_handle.request.angular_z  # 요청받은 각속도

        # 목표 거리에 도달할 때까지 반복
        while True:
            # 현재까지 이동한 거리 업데이트
            self.total_dist += self.calc_diff_pose()
            
            # 남은 거리 계산 (목표 거리 - 현재까지 이동한 거리)
            feedback_msg.remained_dist = goal_handle.request.dist - self.total_dist
            
            # 클라이언트에게 진행 상황 피드백 전송
            goal_handle.publish_feedback(feedback_msg)
            
            # 거북이에게 속도 명령 전송
            self.publisher.publish(msg)

            # quantile_time 지점 통과 여부 확인
            # 예: 목표 5m, quantile_time=0.75이면 3.75m 지점에서 알림
            tmp = feedback_msg.remained_dist - goal_handle.request.dist * self.quantile_time
            tmp = abs(tmp)  # 절댓값으로 변환 (방향 상관없이 거리만 확인)

            # quantile 지점 근처(0.2m 이내)에 도달하면 알림 메시지 출력
            if tmp < 0.2:
                output_msg = "The turtle passes the " + str(self.quantile_time) + " point."
                output_msg = output_msg + " : " + str(tmp)
                self.get_logger().info(output_msg)  # 로그로 진행 상황 알림
            
            # 0.01초 대기 (100Hz 주기로 실행)
            time.sleep(0.01)

            # 남은 거리가 0.2 이하면 목표에 도달했다고 판단하고 종료
            if feedback_msg.remained_dist < 0.2:
                break

        # 액션을 성공으로 표시
        goal_handle.succeed()
        
        # 결과 메시지 생성 및 설정
        result = DistTurtle.Result()
        result.pos_x = self.current_pose.x  # 최종 x 좌표
        result.pos_y = self.current_pose.y  # 최종 y 좌표
        result.pos_theta = self.current_pose.theta  # 최종 각도
        result.result_dist = self.total_dist  # 실제 이동한 거리
        
        # 다음 액션을 위해 변수들 초기화
        self.total_dist = 0
        self.is_first_time = True
        
        return result  # 결과 반환

def main(args=None):
    """메인 함수 - 프로그램 진입점"""
    rp.init(args=args)  # ROS2 초기화
    
    # 멀티스레드 실행기 생성 (여러 노드를 병렬로 실행)
    executor = MultiThreadedExecutor()
    
    # 액션 서버와 구독자 노드 생성
    ac = DistTurtleServer()  # 액션 서버 인스턴스
    sub = TurtleSub_Action(ac_server = ac)  # 위치 구독자 인스턴스
    
    # 실행기에 노드들 추가
    executor.add_node(sub)  # 구독자 노드 추가
    executor.add_node(ac)   # 액션 서버 노드 추가

    try:
        # 노드들을 병렬로 실행 (무한 루프)
        executor.spin()
    finally:
        # 프로그램 종료 시 정리 작업
        executor.shutdown()  # 실행기 종료
        sub.destroy_node()   # 구독자 노드 제거
        ac.destroy_node()    # 액션 서버 노드 제거
        rp.shutdown()        # ROS2 종료

if __name__ == '__main__':
    main()  # 스크립트가 직접 실행될 때 main 함수 호출