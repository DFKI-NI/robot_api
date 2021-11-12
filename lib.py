#!/usr/bin/env python3
from typing import Any, Dict, List
import rospy
import genpy
import actionlib
import re


def find_robot_namespaces() -> List[str]:
    """Return list of robot namespaces by searching published topics for move_base/goal."""
    topics = rospy.get_published_topics()
    robot_namespaces = []  # type: List[str]
    for topic, message_type in topics:
        if message_type == "move_base_msgs/MoveBaseActionGoal":
            match_result = re.match(r'\/(\w+)\/move_base\/goal', topic)
            if match_result:
                robot_namespaces.append(match_result.group(1))
    return robot_namespaces


def _s(count: int, name: str, plural: str='s') -> str:
    """Return name with or without plural ending depending on count."""
    return f"{count} {name}{plural if count != 1 else ''}"


class Component:
    def __init__(self, namespace: str, server_specs: Dict[str, genpy.Message], connect_on_init: bool=False) -> None:
        self._action_clients = {}  # type: Dict[str, actionlib.SimpleActionClient]
        self._connection_results = {}  # type: Dict[actionlib.SimpleActionClient, Any]
        # Add clients to all action servers in server_specs found by published ROS topics.
        for server_name, action_spec in server_specs.items():
            if self._is_topic_of_type(namespace, server_name + "/result", action_spec.__name__ + "Result"):
                action_client = actionlib.SimpleActionClient(namespace + server_name, action_spec)
                # Note: server_name is a key in server_specs and thus unique.
                self._action_clients[server_name] = action_client
                if connect_on_init:
                    self.connect_once(server_name)

    def _is_topic_of_type(self, namespace: str, topic: str, message_type: str) -> bool:
        """Return whether topic is published in namespace using message_type."""
        for check_topic, check_message_type in rospy.get_published_topics(namespace):
            if check_topic == namespace + topic and check_message_type.split('/')[-1] == message_type:
                return True
        return False

    def connect_once(self, server_name: str, timeout: rospy.Duration=rospy.Duration()) -> Any:
        """Connect action client to server_name only once and return its cached result on further calls."""
        action_client = self._action_clients[server_name]
        if action_client not in self._connection_results.keys():
            rospy.logdebug(f"Waiting for {server_name} action server ...")
            self._connection_results[action_client] = action_client.wait_for_server(timeout=timeout)
        return self._connection_results[action_client]
