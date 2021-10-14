#!/usr/bin/env python3
from typing import Any, Callable, Dict, List
import rospy
import re


_executed_functions = {}  # type: Dict[Callable[[], Any], Any]


def execute_once(function: Callable[[], Any]) -> Any:
    """Execute function only once and return its cached result on further calls."""
    # Note: decorator implementation yields mypy untyped decorator error, even when using official documentation at
    #   https://mypy.readthedocs.io/en/stable/generics.html#declaring-decorators
    if function not in _executed_functions.keys():
        _executed_functions[function] = function()
    return _executed_functions[function]


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
