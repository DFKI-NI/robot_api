#!/usr/bin/env python3
from typing import Any, Dict, List, Sequence, Tuple
import os
import time
import re
import rospy
import rosnode
import rosgraph
import genpy
import actionlib
import yaml
from collections import OrderedDict
from robot_api.excepthook import Excepthook


class Storage:
    waypoints = OrderedDict()  # type: Dict[str, Tuple[Sequence[float], Sequence[float]]]
    _next_waypoint = 1

    @classmethod
    def _add_generic_waypoint(cls, position: Sequence[float], orientation: Sequence[float]) -> None:
        """Add (position, orientation) with generic name to list of stored waypoints."""
        while "waypoint" + str(cls._next_waypoint) in cls.waypoints.keys():
            cls._next_waypoint += 1
        cls.waypoints["waypoint" + str(cls._next_waypoint)] = (position, orientation)

    @classmethod
    def _waypoints_to_str(cls) -> str:
        """Convert OrderedDict representation of waypoints to str."""
        return '\n'.join(f"'{waypoint_name}': {waypoint}" for waypoint_name, waypoint in cls.waypoints.items())

    @classmethod
    def _get_custom_waypoint_name(cls, position: Sequence[float], orientation: Sequence[float]) -> str:
        """Return custom name of waypoint (position, orientation) if it exists."""
        for name, waypoint in cls.waypoints.items():
            if not name.startswith("waypoint") and waypoint == (position, orientation):
                return name
        return ""


class Action:
    """
    Generic Action interface.

    Subclass Action to implement a concrete action with a typed parameter interface you specify for its execute()
    method. The first parameter should always be the object which performs the action.

    Instantiate your subclass if you want a specific action instance with parameter values being set. By creating
    this instance with the same parameter interface as you define for the execute() method, you can execute this
    action simply by calling it, e.g., action().
    """
    def __init__(cls, obj: Any, *args: Any, **kwargs: Any) -> None:
        cls.obj = obj
        cls.args = args
        cls.kwargs = kwargs

    def __call__(cls) -> bool:
        if hasattr(cls, "execute"):
            return cls.__getattribute__("execute")(cls.obj, *cls.args, **cls.kwargs)

        raise Excepthook.expect(NotImplementedError("You must subclass Action and implement its execute() method!"))


class ActionlibComponent:
    """
    Robot component, which automatically connects to ROS actionlib servers given in server_specs.
    Unless you connect_on_init, connection will be established on first usage.
    """
    def __init__(cls, namespace: str, server_specs: Dict[str, genpy.Message], connect_on_init: bool=False) -> None:
        cls._action_clients = {}  # type: Dict[str, actionlib.SimpleActionClient]
        cls._connection_results = {}  # type: Dict[actionlib.SimpleActionClient, Any]
        # Add clients to all action servers in server_specs found by published ROS topics.
        for server_name, action_spec in server_specs.items():
            if cls._is_topic_of_type(namespace, server_name + "/result", action_spec.__name__ + "Result"):
                action_client = actionlib.SimpleActionClient(namespace + server_name, action_spec)
                # Note: server_name is a key in server_specs and thus unique.
                cls._action_clients[server_name] = action_client
                if connect_on_init:
                    cls.connect_once(server_name)
            else:
                rospy.logwarn(f"Connecting to actionlib server '{server_name}' failed.")

    @staticmethod
    def _is_topic_of_type(namespace: str, topic: str, message_type: str) -> bool:
        """Return whether topic is published in namespace using message_type."""
        for check_topic, check_message_type in rospy.get_published_topics(namespace):
            if check_topic == namespace + topic and check_message_type.split('/')[-1] == message_type:
                return True
        return False

    def connect_once(cls, server_name: str, timeout: rospy.Duration=rospy.Duration()) -> Any:
        """Connect action client to server_name only once and return its cached result on further calls."""
        action_client = cls._action_clients[server_name]
        if action_client not in cls._connection_results.keys():
            rospy.logdebug(f"Waiting for {server_name} action server ...")
            cls._connection_results[action_client] = action_client.wait_for_server(timeout=timeout)
        return cls._connection_results[action_client]


def _s(count: int, name: str, plural: str='s') -> str:
    """Return name with or without plural ending depending on count."""
    return f"{count} {name}{plural if count != 1 else ''}"


def _init_node() -> None:
    """Initialize ROS node for the current process if not already done."""
    if not rosgraph.is_master_online():
        rospy.logdebug("Waiting for ROS master node to go online ...")
        while not rosgraph.is_master_online():
            time.sleep(1.0)
    if rospy.is_shutdown():
        rospy.logerr("ROS is shutting down.")
        return

    name = f"robot_api_{os.getpid()}"
    # Note: ROS cannot check if a node is initialized, so we have to try.
    #   At least check if we initialized it ourselves before.
    if ('/' + name) not in rosnode.get_node_names():
        try:
            rospy.init_node(name)
            rospy.logdebug(f"ROS node '{name}' initialized.")
        except rospy.ROSException:
            pass


def find_robot_namespaces() -> List[str]:
    """Return list of robot namespaces by searching published topics for move_base/goal."""
    _init_node()
    try:
        topics = rospy.get_published_topics()
    except ConnectionRefusedError as e:
        raise Excepthook.expect(e)

    robot_namespaces = []  # type: List[str]
    for topic, message_type in topics:
        if message_type == "move_base_msgs/MoveBaseActionGoal":
            match_result = re.match(r'\/(\w+)\/move_base\/goal', topic)
            if match_result:
                robot_namespaces.append(match_result.group(1))
    return robot_namespaces


def add_waypoint(name: str, position: Sequence[float], orientation: Sequence[float]) -> None:
    """Add waypoint with name, position [x, y, z] and orientation [x, y, z, w]."""
    _init_node()
    if name in Storage.waypoints.keys():
        rospy.logwarn(f"Overwriting waypoint: {Storage.waypoints[name]}")
    Storage.waypoints[name] = (position, orientation)


def save_waypoints(filepath: str="~/.ros/robot_api_waypoints.txt") -> None:
    """Save waypoints to file, default: '~/.ros/robot_api_waypoints.txt'."""
    _init_node()
    if not Storage.waypoints:
        rospy.logwarn("No waypoints to save.")
        return

    filepath = os.path.expanduser(filepath)
    try:
        with open(filepath, 'w') as text_file:
            for waypoint_name, waypoint in Storage.waypoints.items():
                # Note: Write tuple as list for nicer format.
                text_file.write(f"'{waypoint_name}': {list(waypoint)}\n")
    except Exception:
        rospy.logerr(f"Error while writing to file '{filepath}'!")


def load_waypoints(filepath: str="~/.ros/robot_api_waypoints.txt") -> None:
    """Load waypoints from file, default: '~/.ros/robot_api_waypoints.txt'."""
    _init_node()
    filepath = os.path.expanduser(filepath)
    try:
        with open(filepath, 'r') as text_file:
            lines = [line.strip() for line in text_file.readlines() if line.strip()]
        for line in lines:
            elements = yaml.safe_load(line)
            assert isinstance(elements, dict), f"Invalid format in line: {line}"
            for name, (position, orientation) in elements.items():
                add_waypoint(name, position, orientation)
        rospy.loginfo(f"{_s(len(lines), 'waypoint')} loaded, now {len(Storage.waypoints)} in total.")
    except Exception:
        rospy.logerr(f"Error while reading from file '{filepath}'!")


def print_waypoints() -> None:
    """Output currently stored waypoints with rospy.loginfo()."""
    _init_node()
    rospy.loginfo(f"Available waypoints:\n" + Storage._waypoints_to_str())
