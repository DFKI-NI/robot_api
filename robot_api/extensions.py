#!/usr/bin/env python3
from __future__ import annotations
from typing import Any, Callable, Dict, List, Optional, Tuple
from enum import IntEnum
import re
import rospy
import rosparam
from sensor_msgs.msg import JointState
from robot_api.msg import MoveItMacroAction, MoveItMacroGoal, MoveItMacroResult, FtObserverAction, FtObserverGoal
from robot_api.lib import ActionlibComponent, get_angle_between


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


class Arm(ActionlibComponent):
    ROSLAUNCH_SLEEP_DURATION = 10
    ROBOT_DESCRIPTION_SEMANTIC = "robot_description_semantic"
    MOVEIT_MACROS_TOPIC_NAME = "moveit_macros"
    FT_OBSERVER_TOPIC_NAME = "ft_observer"
    ANGLE_TOLERANCE = 0.01

    def __init__(self, namespace: str, connect_manipulation_on_init: bool) -> None:
        super().__init__(namespace, {
            self.MOVEIT_MACROS_TOPIC_NAME: (MoveItMacroAction,
                f"roslaunch robot_api moveit_macros.launch namespace:='{namespace.strip('/')}'",
                self.ROSLAUNCH_SLEEP_DURATION),
            self.FT_OBSERVER_TOPIC_NAME: (FtObserverAction,)
        }, connect_manipulation_on_init)
        self._pose_joint_values = self._get_pose_joint_values()
        self.pose_names = list(self._pose_joint_values.keys())

    @staticmethod
    def _parse(pattern: str, string: str) -> str:
        """Return first occurrence of pattern in string, or raise AssertionError if pattern cannot be found."""
        match_result = re.search(pattern, string)
        assert match_result is not None, f"Error: Cannot parse '{string}' from '{pattern}'!"
        return match_result.group(1)

    def _get_pose_joint_values(self) -> Dict[str, Dict[str, float]]:
        """Get joint values from semantic robot description parameter used for arm poses."""
        params = rosparam.list_params(self._namespace)
        # If default param name exists, use it.
        if self._namespace + self.ROBOT_DESCRIPTION_SEMANTIC in params:
            param = rosparam.get_param(self._namespace + self.ROBOT_DESCRIPTION_SEMANTIC)
        else:
            # Otherwise search for param which ends with '_semantic', according to planning_context.launch.
            for param in params:
                if param.endswith("_semantic"):
                    break
            else:
                return {}

        # Collect all joint values from group states associated with group "arm".
        group_tokens: List[Tuple[str, str]] = re.findall(r'<group_state\s+([\'\"\w\s=]+)>(.*?)</group_state>',
            param, re.DOTALL)
        return {
            self._parse(r'name=[\'\"](\w+)[\'\"]', token): {
                self._parse(r'name=[\'\"](.+?)[\'\"]', line): float(self._parse(r'value=[\'\"](.+?)[\'\"]', line))
                for line in content.split('\n') if line.strip().startswith("<joint ")
            }
            for token, content in group_tokens if "group='arm'" in token or 'group="arm"' in token
        }

    def _call_moveit_macro(self, goal_type: str, goal_name: str,
            done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        """Call MoveItMacro with goal_type and goal_name. Optionally, call done_cb() afterwards if given."""
        if not self._connect(self.MOVEIT_MACROS_TOPIC_NAME):
            rospy.logerr("Did you 'roslaunch mobipick_pick_n_place moveit_macros.launch' with correct 'namespace'?")
            return None

        goal = MoveItMacroGoal(type=goal_type, name=goal_name)
        return self._action_clients[self.MOVEIT_MACROS_TOPIC_NAME].send_goal_and_wait(goal) if done_cb is None \
            else self._action_clients[self.MOVEIT_MACROS_TOPIC_NAME].send_goal(goal, done_cb)

    def execute(self, action_name: str, done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        """Execute moveit_macro named action_name. Optionally, call done_cb() afterwards if given."""
        return self._call_moveit_macro("function", action_name, done_cb)

    def move(self, pose_name: str, done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        """Move arm to pose named pose_name. Optionally, call done_cb() afterwards if given."""
        return self._call_moveit_macro("target", pose_name, done_cb)

    def observe_force_torque(self, threshold: float, timeout: float) -> bool:
        """Call force torque observer with given threshold and timeout. Return whether successful."""
        if not self._connect(self.FT_OBSERVER_TOPIC_NAME):
            rospy.logerr("Did you launch the ft_observer node?")
            return False

        goal = FtObserverGoal(threshold=threshold, timeout=timeout)
        action_client = self._action_clients[self.FT_OBSERVER_TOPIC_NAME]
        action_client.send_goal(goal)
        action_client.wait_for_result()
        # Note: http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
        return int(action_client.get_state()) == 3

    def get_pose_name(self, angle_tolerance=ANGLE_TOLERANCE, timeout: Optional[float]=None) -> Optional[str]:
        """Return the pose name if the robot arm is currently in one of the known poses."""
        joint_state: JointState = rospy.wait_for_message(self._namespace + "joint_states", JointState, timeout)
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
