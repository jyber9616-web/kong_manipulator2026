# ros2 action send_goal /fibonacci_server \
# user_interface/action/Fibonacci "{step: 10}" --feedback

import sys

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from user_interface.action import Fibonacci
from user_interface.action._fibonacci import Fibonacci_GetResult_Response


class Action_client(Node):
    def __init__(self):
        super().__init__("action_client")

        self.action_client = ActionClient(
            self,
            Fibonacci,
            "fibonacci_server",
        )

        self.goal_handle: ClientGoalHandle | None = None
        self.done = False

    def send_goal(self, step: str):
        goal_msg = Fibonacci.Goal()
        goal_msg.step = int(step)

        while not self.action_client.wait_for_server(timeout_sec=1):
            self.get_logger().info("fibonacci server is not available!!")

        self.future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):
        self.goal_handle = future.result()  # type: ignore

        if not self.goal_handle.accepted:
            self.get_logger().info("goal rejected!!")
            self.goal_handle = None
            self.done = True
            return

        self.get_logger().info("goal accepted!!")

        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

        self.get_logger().info("end of goal response callback function!!")

    def feedback_callback(
        self,
        msg: Fibonacci.Impl.FeedbackMessage,
    ):
        feedback: Fibonacci.Feedback = msg.feedback
        self.get_logger().info(f"{list(feedback.temp_seq)}")

    def get_result_callback(self, future: Future):
        result: Fibonacci_GetResult_Response = (
            future.result()  # type: ignore
        )
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"result: {list(result.result.seq)}")
        elif result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("aborted!!")
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("canceled!!")
        self.done = True

    def cancel_goal(self):
        if self.goal_handle is None:
            print("취소할 Goal이 없습니다.")
            return
        print("Goal 취소 요청을 보냅니다.")
        cancel_future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(
            self,
            cancel_future,
            timeout_sec=3.0,
        )
        if not cancel_future.done():
            print("Goal 취소 응답 시간이 초과되었습니다.")
            return
        cancel_response = cancel_future.result()
        if cancel_response is not None and cancel_response.goals_canceling:
            print("Goal 취소 요청이 승인되었습니다.")
        else:
            print("Goal 취소 요청이 승인되지 않았습니다.")


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = Action_client()
    if len(sys.argv) != 2:
        print("사용법 : ros2 run [package] [node] [step: int]")
        node.destroy_node()
        rclpy.try_shutdown()
        return

    node.send_goal(sys.argv[1])

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("키보드 인터럽트")
        node.cancel_goal()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
