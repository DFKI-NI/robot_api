#!/usr/bin/env python3
from typing import Any, Callable, Dict, Optional
import rospy
import genpy
import actionlib
from mobipick_pick_n_place.msg import MoveItMacroAction, MoveItMacroGoal, MoveItMacroResult, \
    FtObserverAction, FtObserverGoal
from robot_api.lib import execute_once


class Extension:
    def __init__(self, namespace: str, server_specs: Dict[str, genpy.Message], connect_on_init: bool=False) -> None:
        self._action_clients = {}  # type: Dict[str, actionlib.SimpleActionClient]
        # Add clients to all action servers in server_specs found by published ROS topics.
        for server_name, action_spec in server_specs.items():
            if self._is_topic_of_type(namespace, server_name + "/result", action_spec.__name__ + "Result"):
                action_client = actionlib.SimpleActionClient(namespace + server_name, action_spec)
                # Note: server_name is a key in server_specs and thus unique.
                self._action_clients[server_name] = action_client
                if connect_on_init:
                    self._connect(server_name)

    def _connect(self, server_name: str) -> Any:
        rospy.logdebug(f"Waiting for {server_name} action server ...")
        return self._action_clients[server_name].wait_for_server()

    def _is_topic_of_type(self, namespace: str, topic: str, message_type: str) -> bool:
        """Return whether topic is published in namespace using message_type."""
        for check_topic, check_message_type in rospy.get_published_topics(namespace):
            if check_topic == namespace + topic and check_message_type.split('/')[-1] == message_type:
                return True
        return False


class Arm(Extension):
    def __init__(self, namespace: str, connect_manipulation_on_init: bool) -> None:
        super().__init__(namespace, {
            "moveit_macros": MoveItMacroAction,
            "ft_observer": FtObserverAction
        }, connect_manipulation_on_init)

    def execute(self, action_name: str, action_type: str="function",
            done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        """Execute moveit_macro of action_type with action_name. Optionally, call done_cb() afterwards if given."""
        if not self._connect("moveit_macros"):
            rospy.logerr(f"Cannot execute MoveIt action.{' ROS is shutting down.' if rospy.is_shutdown() else ''}")
            return None

        goal = MoveItMacroGoal()
        goal.type = action_type
        goal.name = action_name
        if done_cb is None:
            return self._action_clients["moveit_macros"].send_goal_and_wait(goal)
        else:
            return self._action_clients["moveit_macros"].send_goal(goal, done_cb)

    def move(self, goal_name: str, done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        """Move arm to pose with goal_name. Optionally, call done_cb() afterwards if given."""
        rospy.logdebug(f"Sending moveit_macro goal '{goal_name}' ...")
        return self.execute(goal_name, "target", done_cb)

    def get_result(self, action_name: str, done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        """Execute action_name and return its result (not state). Optionally, call done_cb() afterwards if given."""
        self.execute(action_name, "function", done_cb)
        return self._action_clients["moveit_macros"].get_result()

    def observe_user_interaction(self) -> Any:
        """Observe and return state."""
        self._connect("ft_observer")
        goal = FtObserverGoal()
        goal.threshold = 5.0
        goal.timeout = 30
        action_client = self._action_clients["ft_observer"]
        action_client.send_goal(goal)
        action_client.wait_for_result()
        # Note: http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
        return action_client.get_state() == 3
