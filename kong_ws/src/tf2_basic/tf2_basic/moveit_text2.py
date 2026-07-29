"""MoveItPy로 OpenManipulator-X의 arm과 gripper를 실행한다."""

import os
import sys
import time

import rclpy
from moveit.planning import MoveItPy


def exit_without_moveit_destructor(exit_code: int) -> None:
    """MoveItPy Jazzy 종료 충돌을 피해 일회성 프로세스를 끝낸다."""
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(exit_code)


def plan_and_execute(
    moveit: MoveItPy,
    component,
    configuration_name: str,
    controller_name: str,
    execute_motion: bool,
) -> bool:
    """현재 상태에서 named state까지 계획한 뒤 지정 controller로 실행한다."""
    component.set_start_state_to_current_state()
    component.set_goal_state(configuration_name=configuration_name)

    print(f"경로 계획 중: {configuration_name}")
    plan_result = component.plan()
    if not plan_result:
        print(f"경로 계획 실패: {configuration_name}")
        return False

    if not execute_motion:
        print(f"경로 계획 성공(실행 생략): {configuration_name}")
        return True

    print(f"경로 실행 중: {configuration_name}")
    moveit.execute(
        plan_result.trajectory,
        controllers=[controller_name],
    )
    return True


def main() -> None:
    """arm을 home으로 이동하고 gripper를 열고 닫는다."""
    rclpy.init()
    moveit = None
    arm = None
    gripper = None
    try:
        moveit = MoveItPy(node_name="open_manipulator_moveit_py")
        execute_motion = os.environ.get("MOVEIT_TEST_PLAN_ONLY") != "1"
        arm = moveit.get_planning_component("arm")
        gripper = moveit.get_planning_component("gripper")

        if not plan_and_execute(
            moveit,
            arm,
            configuration_name="home",
            controller_name="arm_controller",
            execute_motion=execute_motion,
        ):
            exit_without_moveit_destructor(1)

        if execute_motion:
            time.sleep(0.5)
        for goal_name in ("open", "close", "open"):
            if not plan_and_execute(
                moveit,
                gripper,
                configuration_name=goal_name,
                controller_name="gripper_controller",
                execute_motion=execute_motion,
            ):
                exit_without_moveit_destructor(1)
            if execute_motion:
                time.sleep(0.7)
        print("MoveItPy arm·gripper 실습 완료")
        exit_without_moveit_destructor(0)
    except KeyboardInterrupt:
        exit_without_moveit_destructor(130)
    finally:
        # PlanningComponent가 MoveItPy의 C++ 객체를 참조하므로 역순으로 해제한다.
        if gripper is not None:
            del gripper
        if arm is not None:
            del arm
        if moveit is not None:
            del moveit
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
