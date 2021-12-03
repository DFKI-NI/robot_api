#!/usr/bin/env python3
from __future__ import annotations
from typing import Any, Callable, List, Optional
import re
import rospy
import rosparam
from mobipick_pick_n_place.msg import MoveItMacroAction, MoveItMacroGoal, MoveItMacroResult, \
    FtObserverAction, FtObserverGoal
from robot_api.lib import _is_topic_of_type, Action, ActionlibComponent


class ArmMoveItMacroAction(Action):
    @staticmethod
    def execute(arm: Arm, goal_type: str, goal_name: str,
            done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        if not arm.connect("moveit_macros"):
            return None

        goal = MoveItMacroGoal(type=goal_type, name=goal_name)
        if done_cb is None:
            return arm._action_clients["moveit_macros"].send_goal_and_wait(goal)
        else:
            return arm._action_clients["moveit_macros"].send_goal(goal, done_cb)


class ArmForceTorqueObserverAction(Action):
    @staticmethod
    def execute(arm: Arm, threshold: float, timeout: float) -> bool:
        if not arm.connect("ft_observer"):
            return False

        goal = FtObserverGoal(threshold=threshold, timeout=timeout)
        action_client = arm._action_clients["ft_observer"]
        action_client.send_goal(goal)
        action_client.wait_for_result()
        # Note: http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
        return int(action_client.get_state()) == 3


class Arm(ActionlibComponent):
    ROBOT_DESCRIPTION_SEMANTIC = "robot_description_semantic"

    def __init__(self, namespace: str, connect_manipulation_on_init: bool) -> None:
        super().__init__(namespace, {
            "moveit_macros": MoveItMacroAction,
            "ft_observer": FtObserverAction
        }, connect_manipulation_on_init)

    def execute(self, action_name: str, done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        """Execute moveit_macro named action_name. Optionally, call done_cb() afterwards if given."""
        return ArmMoveItMacroAction.execute(self, "function", action_name, done_cb)

    def get_result(self, action_name: str, done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        """Execute action_name and return its result (not state). Optionally, call done_cb() afterwards if given."""
        ArmMoveItMacroAction.execute(self, "function", action_name, done_cb)
        return self._action_clients["moveit_macros"].get_result()

    def move(self, goal_name: str, done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        """Move arm to pose named goal_name. Optionally, call done_cb() afterwards if given."""
        rospy.logdebug(f"Sending moveit_macro goal '{goal_name}' ...")
        return ArmMoveItMacroAction.execute(self, "target", goal_name, done_cb)

    def get_state_names(self) -> List[str]:
        """Get group state names from semantic robot description parameter used for arm poses."""
        params = rosparam.list_params(self.namespace)
        # If default param name exists, use it.
        if self.namespace + self.ROBOT_DESCRIPTION_SEMANTIC in params:
            param = rosparam.get_param(self.namespace + self.ROBOT_DESCRIPTION_SEMANTIC)
        else:
            # Otherwise search for param which ends with '_semantic', according to planning_context.launch.
            for param in params:
                if param.endswith("_semantic"):
                    break
            else:
                return []

        # Collect all names from group states associated with group "arm".
        tokens = re.findall(r'<group_state ([= \"\w]+)>', param)
        names = []  # type: List[str]
        for token in tokens:
            if "group='arm'" in token or 'group="arm"' in token:
                match_result = re.search(r'name=["\'](\w+)["\']', token)
                if match_result:
                    names.append(match_result.group(1))
        return names
