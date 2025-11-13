from __future__ import annotations
from typing import Any, Callable, Dict, List, Optional, Tuple
from enum import IntEnum
import os
import re

from sensor_msgs.msg import JointState

from robot_api.lib import get_angle_between
from robot_api.ros_wrapper import get_ros_wrapper, RosWrapperInterface

try:
    ros_version = os.environ["ROS_VERSION"]
    if ros_version == "1":
        from robot_api_msgs.msg import MoveItMacroAction, MoveItMacroGoal, MoveItMacroResult, FtObserverAction, FtObserverGoal
        from control_msgs.msg import GripperCommandAction
    elif ros_version == "2":
        from robot_api_msgs.action import MoveItMacro as MoveItMacroAction
        MoveItMacroGoal = MoveItMacroAction.Goal
        MoveItMacroResult = MoveItMacroAction.Result
        from robot_api_msgs.action import FtObserver as FtObserverAction
        FtObserverGoal = FtObserverAction.Goal
        from control_msgs.action import ParallelGripperCommand as GripperCommandAction
    else:
        raise ImportError(f"Unsupported ROS_VERSION: {ros_version}")
except KeyError:
    raise ImportError("ROS_VERSION environment variable not set. Please source your ROS setup file.")


_ros_wrapper: RosWrapperInterface = get_ros_wrapper()


class TaskStage(IntEnum):
    # Note: This must match SetTask.msg.
    MOVE_TO_NAMED_POSE = 10
    MOVE_TO_POSE = 11
    MOVE_RELATIVE = 12
    MOVE_TO_JOINT_POSE = 13
    MOVE_GRIPPER_TO_NAMED_POSE = 20
    MOVE_GRIPPER_TO_POSE = 21
    MOVE_GRIPPER_RELATIVE = 22
    GRASP_CONTAINER = 30
    PLACE_CONTAINER = 40
    CONNECT_STATE = 100
    ADD_PREDICATE_STATE = 101


class Arm():
    ROSLAUNCH_SLEEP_DURATION = 10
    ROBOT_DESCRIPTION_SEMANTIC = "robot_description_semantic"
    ANGLE_TOLERANCE = 0.01

    def __init__(
        self,
        namespace: str,
        connect_manipulation_on_init: bool,
        group_name: str = "arm",
    ) -> None:
        _ros_wrapper.init_action_server(
            namespace,
            {
                _ros_wrapper.get_moveit_topic_name(): (
                    MoveItMacroAction,
                    _ros_wrapper.launch_moveit_macros_command(),
                    self.ROSLAUNCH_SLEEP_DURATION,
                ),
                _ros_wrapper.get_ft_observer_topic_name(): (FtObserverAction,),
            },
            connect_manipulation_on_init,
        )
        self._pose_joint_values = self._get_pose_joint_values()
        self.pose_names = list(self._pose_joint_values.keys())
        self.group_name = group_name

    @staticmethod
    def _parse(pattern: str, string: str) -> str:
        """
        Return first occurrence of pattern in string,
         or raise AssertionError if pattern cannot be found.
        """
        match_result = re.search(pattern, string)
        assert (
            match_result is not None
        ), f"Error: Cannot parse '{string}' from '{pattern}'!"
        return match_result.group(1)

    def _get_pose_joint_values(self) -> Dict[str, Dict[str, float]]:
        """Get joint values from semantic robot description parameter used for arm poses."""
        params = _ros_wrapper.list_params(_ros_wrapper._namespace)
        # If default param name exists, use it.
        if _ros_wrapper._namespace + self.ROBOT_DESCRIPTION_SEMANTIC in params:
            param = _ros_wrapper.get_param(
                _ros_wrapper._namespace + self.ROBOT_DESCRIPTION_SEMANTIC
            )
        else:
            # Otherwise search for param which ends with '_semantic',
            #  according to planning_context.launch.
            for param in params:
                if param.endswith("_semantic"):
                    break
            else:
                return {}

        # Collect all joint values from group states associated with group "arm".
        group_tokens: List[Tuple[str, str]] = re.findall(
            r"<group_state\s+([\'\"\w\s=]+)>(.*?)</group_state>", param, re.DOTALL
        )
        return {
            self._parse(r"name=[\'\"](\w+)[\'\"]", token): {
                self._parse(r"name=[\'\"](.+?)[\'\"]", line): float(
                    self._parse(r"value=[\'\"](.+?)[\'\"]", line)
                )
                for line in content.split("\n")
                if line.strip().startswith("<joint ")
            }
            for token, content in group_tokens
            if "group='arm'" in token or 'group="arm"' in token
        }

    def _call_moveit_macro(
        self,
        goal_type: str,
        goal_name: str,
        done_cb: Optional[Callable[[int, MoveItMacroResult], Any]] = None,
    ) -> Any:
        """
        Call MoveItMacro with goal_type and goal_name.
         Return the moveit_macro action server's result.
        If done_cb is given, make this an asynchronous action and call done_cb() when done.
        """
        if not _ros_wrapper._connect_to_action_server(_ros_wrapper.get_moveit_topic_name()):
            _ros_wrapper.log(
                "Did you launch moveit_macros"
                " with correct 'namespace'?",
                level="error",
            )
            return None

        goal = MoveItMacroGoal()
        goal.type = goal_type
        goal.name = goal_name
        return (
            _ros_wrapper.send_goal_and_wait(_ros_wrapper.get_moveit_topic_name(), goal)
            if done_cb is None
            else _ros_wrapper.send_goal(
                _ros_wrapper.get_moveit_topic_name(),
                goal, done_cb
            )
        )

    def execute(
        self,
        action_name: str,
        done_cb: Optional[Callable[[int, MoveItMacroResult], Any]] = None,
    ) -> Any:
        """
        Execute moveit_macro named action_name.
         Return the moveit_macro action server's result.
        If done_cb is given, make this an asynchronous action and call done_cb() when done.
        """
        return self._call_moveit_macro("function", action_name, done_cb)

    def move(
        self,
        pose_name: str,
        done_cb: Optional[Callable[[int, MoveItMacroResult], Any]] = None,
    ) -> Any:
        """
        Move arm to pose named pose_name. Return the moveit_macro action server's result.
        If done_cb is given, make this an asynchronous action and call done_cb() when done.
        """
        return self._call_moveit_macro("target", pose_name, done_cb)

    def observe_force_torque(self, threshold: float, timeout: float) -> bool:
        """
        Call force torque observer with given threshold and timeout.
         Return whether successful.
        """
        if not _ros_wrapper._connect_to_action_server(_ros_wrapper.get_ft_observer_topic_name()):
            _ros_wrapper.log("Did you launch the ft_observer node?", level="error")
            return False

        goal = FtObserverGoal()
        goal.threshold = threshold
        goal.timeout = timeout
        result = _ros_wrapper.send_goal_and_wait(_ros_wrapper.get_ft_observer_topic_name(), goal, timeout)
        return result.catched if result is not None else False

    def get_pose_name(
        self, angle_tolerance=ANGLE_TOLERANCE, timeout: Optional[float] = None
    ) -> Optional[str]:
        """Return the pose name if the robot arm is currently in one of the known poses."""
        joint_state: JointState = _ros_wrapper.wait_for_message(
            _ros_wrapper._namespace + "joint_states", JointState, timeout
        )
        for pose_name, joints in self._pose_joint_values.items():
            for joint_name, value in joints.items():
                if joint_name not in joint_state.name:
                    break
                angle = joint_state.position[joint_state.name.index(joint_name)]
                if abs(get_angle_between(angle, value)) > angle_tolerance:
                    break
            else:
                return pose_name
        return None


class Gripper():
    def __init__(self, namespace: str, connect_manipulation_on_init: bool = False):
        _ros_wrapper.init_action_server(
            namespace,
            {_ros_wrapper.get_gripper_topic_name(): (GripperCommandAction,)},
            connect_manipulation_on_init,
        )

    def open(self):
        if _ros_wrapper.get_gripper_topic_name() not in _ros_wrapper._action_clients:
            _ros_wrapper._connect_to_action_server(_ros_wrapper.get_gripper_topic_name())
        _ros_wrapper.send_goal_and_wait(_ros_wrapper.get_gripper_topic_name(), _ros_wrapper.create_open_gripper_goal())

    def close(self):
        if _ros_wrapper.get_gripper_topic_name() not in _ros_wrapper._action_clients:
            _ros_wrapper._connect_to_action_server(_ros_wrapper.get_gripper_topic_name())
        _ros_wrapper.send_goal_and_wait(_ros_wrapper.get_gripper_topic_name(), _ros_wrapper.create_close_gripper_goal())
