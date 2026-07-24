import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Color, Pose


class Move_tutle(Node):
    def __init__(self):
        super().__init__("move_turtle")  # 노드 이름
        # timer 등록
        self.create_timer(0.1, self.timer_callback)
        self.declare_parameter("option", "first")
        self.option = self.get_parameter("option").get_parameter_value().string_value
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Pose, "pose", self.pose_callback, 10)
        self.create_subscription(Color, "color_sensor", self.color_callback, 10)
        self.count = 0.0
        self.pose = Pose()
        self.color = Color()

    def timer_callback(self):
        msg = Twist()  # DDS 에 보낼 객체 초기화
        if self.option == "first":
            msg.linear.x = 0.0 + self.count
            msg.angular.z = 1.0

            # 별 그리기 상태 관리
            if not hasattr(self, "state"):
                self.state = "MOVE"  # MOVE 또는 TURN
                self.step_count = 0

            if self.state == "MOVE":
                msg.linear.x = 2.0
                msg.angular.z = 0.0
                self.step_count += 1
                # 2초 동안 직진 (0.1초 timer * 20 = 2초)
                if self.step_count >= 20:
                    self.state = "TURN"
                    self.step_count = 0

            elif self.state == "TURN":
                msg.linear.x = 0.0
                # 5각 별 회전 각도 (144도 ≈ 2.513 rad)
                msg.angular.z = 2.513
                self.step_count += 1
                # 1초 동안 회전
                if self.step_count >= 10:
                    self.state = "MOVE"
                    self.step_count = 0

            self.pub.publish(msg)  # DDS 로 보내는 기능 수행
            # AI 활용해서 msg 재미 있게 움직이도록 수정 self.pose, self.color
            # 별표로 움직이기 지그재그로 움직이기 ...
            self.count += 0.01
            if self.count > 3.0:
                self.count = 0.0
        if self.option == "second":
            msg = Twist()

            x = self.pose.x
            y = self.pose.y
            theta = self.pose.theta  # -pi ~ pi 범위 (라디안)

            # 1. 각 벽에 접근했는지 판단
            near_right = x > 9.5
            near_left = x < 1.5
            near_top = y > 9.5
            near_bottom = y < 1.5

            # 2. 벽에 접근 중이고, "벽 쪽을 바라보고 있을 때만" 회전 제어
            # 오른쪽 벽(x > 9.5): 오른쪽(-pi/2 ~ pi/2)을 바라볼 때 회전
            if near_right and (-math.pi / 2 < theta < math.pi / 2):
                msg.linear.x = 0.5
                msg.angular.z = 2.0  # 반시계 방향으로 틀어서 안쪽으로 복귀

            # 왼쪽 벽(x < 1.5): 왼쪽(pi/2 이상 또는 -pi/2 이하)을 바라볼 때 회전
            elif near_left and (theta > math.pi / 2 or theta < -math.pi / 2):
                msg.linear.x = 0.5
                msg.angular.z = 2.0

            # 위쪽 벽(y > 9.5): 위쪽(0 ~ pi)을 바라볼 때 회전
            elif near_top and (0 < theta < math.pi):
                msg.linear.x = 0.5
                msg.angular.z = 2.0

            # 아래쪽 벽(y < 1.5): 아래쪽(-pi ~ 0)을 바라볼 때 회전
            elif near_bottom and (-math.pi < theta < 0):
                msg.linear.x = 0.5
                msg.angular.z = 2.0

            # 3. 벽이 없거나 이미 안쪽을 바라보고 있다면 정상 진행
            else:
                msg.linear.x = 2.0
                msg.angular.z = 0.5

            self.pub.publish(msg)

    def pose_callback(self, msg: Pose):
        self.pose = msg

    def color_callback(self, msg: Color):
        self.color = msg


def main(args=None):
    rclpy.init(args=args)  # rmw 활성화
    node = Move_tutle()
    try:
        rclpy.spin(node)  # 블럭 (무한 루프)
    except KeyboardInterrupt:
        print("키보드 인터럽트")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()