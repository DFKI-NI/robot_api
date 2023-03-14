from __future__ import annotations
from typing import (
    Any,
    Dict,
    List,
    Mapping,
    Optional,
    Sequence,
    Tuple,
    Union,
    get_args,
    get_origin,
)
import os
import shlex
import subprocess
import time
import math
import re
import rospy
import rosnode
import rosgraph
import genpy
import tf
import actionlib
import yaml
from collections import OrderedDict
from geometry_msgs.msg import Point, Pose, Quaternion
from robot_api.excepthook import Excepthook


def _s(count: int, name: str, plural: str = "s") -> str:
    """Return name with or without plural ending depending on count."""
    return f"{count} {name}{plural if count != 1 else ''}"


def _init_node() -> None:
    """Initialize ROS node for the current process if not already done."""
    if not rosgraph.is_master_online():
        print("Waiting for ROS master node to go online ...")
        while not rosgraph.is_master_online():
            time.sleep(1.0)
    if rospy.is_shutdown():
        rospy.logerr("ROS is shutting down.")
        return

    name = f"robot_api_{os.getpid()}"
    # Note: ROS cannot check if a node is initialized, so we have to try.
    #   At least check if we initialized it ourselves before.
    if ("/" + name) not in rosnode.get_node_names():
        try:
            rospy.init_node(name)
            rospy.logdebug(f"ROS node '{name}' initialized.")
        except rospy.ROSException:
            pass


def _is_topic_of_type(namespace: str, topic: str, message_type: str) -> bool:
    """Return whether topic is published in namespace using message_type."""
    for check_topic, check_message_type in rospy.get_published_topics(namespace):
        if (
            check_topic == namespace + topic
            and check_message_type.split("/")[-1] == message_type
        ):
            return True
    return False


def _is_topic_with_suffix(namespace: str, topic: str, suffix: str) -> bool:
    """Return whether topic is published in namespace and its type ends with suffix."""
    for check_topic, check_message_type in rospy.get_published_topics(namespace):
        if check_topic == namespace + topic and check_message_type.endswith(suffix):
            return True
    return False


def _execute(command: str, sleep_duration: int = 0) -> None:
    """Execute command in a new subprocess and sleep for sleep_duration seconds."""
    rospy.loginfo(command)
    subprocess.Popen(shlex.split(command), stdout=subprocess.DEVNULL)
    if sleep_duration:
        time.sleep(sleep_duration)


class TuplePose:
    """Helper class for handling geometry_msgs/Pose."""

    @staticmethod
    def from_pose(
        pose: Pose,
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
        p, q = pose.position, pose.orientation
        return ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    @staticmethod
    def from_sequence_tuple(
        pose: Tuple[Sequence[float], Sequence[float]]
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
        position, orientation = pose
        assert len(position) == 3, "First parameter must be a vector of len 3."
        assert len(orientation) == 4, "Second parameter must be a quaternion of len 4."
        return (
            (position[0], position[1], position[2]),
            (orientation[0], orientation[1], orientation[2], orientation[3]),
        )

    @staticmethod
    def to_pose(pose: Tuple[Sequence[float], Sequence[float]]) -> Pose:
        return Pose(Point(*pose[0]), Quaternion(*pose[1]))

    @staticmethod
    def to_str(pose: Tuple[Sequence[float], Sequence[float]]) -> str:
        # Note: Use list representations to easily parse with yaml afterwards.
        return f"[{list(pose[0])}, {list(pose[1])}]"


class Storage:
    """Helper class for automatically generating navigation waypoints."""

    waypoints: Dict[
        str, Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]
    ] = OrderedDict()
    _next_waypoint = 1

    @classmethod
    def _add_generic_waypoint(
        cls, pose: Tuple[Sequence[float], Sequence[float]]
    ) -> None:
        """Add pose with a generic name to stored waypoints."""
        while "waypoint" + str(cls._next_waypoint) in cls.waypoints.keys():
            cls._next_waypoint += 1
        cls.waypoints[
            "waypoint" + str(cls._next_waypoint)
        ] = TuplePose.from_sequence_tuple(pose)

    @classmethod
    def _waypoints_to_str(cls) -> str:
        """Convert internal representation of waypoints to str."""
        return "\n".join(
            f"'{waypoint_name}': {TuplePose.to_str(waypoint)}"
            for waypoint_name, waypoint in cls.waypoints.items()
        )

    @classmethod
    def _get_custom_waypoint_name(
        cls, pose: Tuple[Sequence[float], Sequence[float]]
    ) -> str:
        """Return custom name of waypoint pose == (position, orientation) if it exists."""
        tuple_pose = TuplePose.from_sequence_tuple(pose)
        for name, waypoint in cls.waypoints.items():
            if re.match(r"waypoint\d+", name) is None and waypoint == tuple_pose:
                return name
        return ""


class ActionlibComponent:
    """
    Robot component, which automatically connects to ROS actionlib servers given in server_specs.
    Unless you connect_on_init, connection will be established lazily on first usage.
    """

    def __init__(
        self,
        namespace: str,
        server_specs: Dict[
            str, Union[Tuple[genpy.Message], Tuple[genpy.Message, str, int]]
        ],
        connect_on_init: bool = False,
    ) -> None:
        self._namespace = namespace
        assert (
            server_specs
        ), "Error: You must init an ActionlibComponent with an item in server_specs."
        self._server_specs = server_specs
        self._action_clients: Dict[str, actionlib.SimpleActionClient] = {}
        self._last_server_name: Optional[str] = None
        if connect_on_init:
            for server_name in server_specs.keys():
                self._connect(server_name)

    def _has_actionlib_result(self, server_name: str) -> bool:
        """Return whether there exists an actionlib result topic for server_name."""
        return _is_topic_with_suffix(self._namespace, server_name + "/result", "Result")

    def _connect(
        self, server_name: str, timeout: rospy.Duration = rospy.Duration()
    ) -> bool:
        """
        Connect action client to server_name if not yet successfully done.
        Return whether the connection succeeded.
        """
        self._last_server_name = server_name
        if not server_name in self._action_clients.keys():
            server_spec = self._server_specs[server_name]
            has_actionlib_result = self._has_actionlib_result(server_name)
            if not has_actionlib_result:
                if len(server_spec) == 3:
                    _execute(*server_spec[1:])
                    has_actionlib_result = self._has_actionlib_result(server_name)
                if not has_actionlib_result:
                    rospy.logerr(f"Server '{server_name}' not found by topic.")
                    return False

            if not server_name in self._server_specs.keys():
                rospy.logerr(
                    f"Cannot connect to server '{server_name}' with unknown type."
                )
                return False

            action_client = actionlib.SimpleActionClient(
                self._namespace + server_name, server_spec[0]
            )
            if not action_client.wait_for_server(timeout=timeout):
                rospy.logerr(
                    f"Timeout while trying to connect to server '{server_name}'."
                    f"{' ROS is shutting down.' if rospy.is_shutdown() else ''}"
                )
                return False

            # Note: server_name is a key in server_specs and thus unique.
            self._action_clients[server_name] = action_client
        return True

    def get_result(self, server_name: Optional[str] = None) -> Any:
        """Return result from action client connected to server_name. Use the last connected server_name by default."""
        if server_name is None:
            server_name = self._last_server_name
            if server_name is None:
                rospy.logerr("Cannot get result before connecting to any server.")
                return None
        if server_name not in self._action_clients.keys():
            rospy.logerr(f"Cannot get result from server '{server_name}'.")
            return None

        return self._action_clients[server_name].get_result()


def is_instance(obj: object, type_or_generic: Any) -> bool:
    """Return whether obj's and its potential elements' types match type_or_generic."""
    origin = get_origin(type_or_generic)
    if origin:
        args = get_args(type_or_generic)
        if origin is Union:
            return any(is_instance(obj, arg) for arg in args)
        if issubclass(origin, tuple) and (len(args) < 2 or args[1] is not Ellipsis):
            return (
                isinstance(obj, origin)
                and len(obj) == len(args)
                and all(is_instance(element, arg) for element, arg in zip(obj, args))
            )
        if issubclass(origin, Sequence):
            return isinstance(obj, origin) and (
                not args or all(is_instance(element, args[0]) for element in obj)
            )
        if issubclass(origin, Mapping):
            return isinstance(obj, origin) and (
                not args
                or all(
                    is_instance(key, args[0]) and is_instance(value, args[1])
                    for key, value in obj.items()
                )
            )
        raise NotImplementedError(
            f"is_instance() is not implemented for {type_or_generic}!"
        )
    return isinstance(obj, type_or_generic)


def get_at(args: Any, index: int, type_or_generic: Any) -> Any:
    """Return element in args at index if its type matches type_or_generic, else None."""
    return (
        args[index]
        if isinstance(args, Sequence)
        and len(args) > index
        and is_instance(args[index], type_or_generic)
        else None
    )


def get_angle_between(source: float, target: float) -> float:
    """Return angle from source to target in [-pi, pi)."""
    angle = target - source
    while angle < -math.pi:
        angle += 2 * math.pi
    while angle >= math.pi:
        angle -= 2 * math.pi
    return angle


def get_pose_name(
    pose: Tuple[Sequence[float], Sequence[float]],
    poses: Mapping[str, Tuple[Sequence[float], Sequence[float]]] = Storage.waypoints,
    xy_tolerance=math.inf,
    yaw_tolerance=math.inf,
) -> Optional[str]:
    """Return the name of the closest of poses to pose within the given tolerances."""
    position, orientation = pose
    _, _, yaw = tf.transformations.euler_from_quaternion(orientation)
    pose_name: Optional[str] = None
    min_yaw_distance = math.pi
    for check_name, (check_position, check_orientation) in poses.items():
        _, _, check_yaw = tf.transformations.euler_from_quaternion(check_orientation)
        xy_distance = math.dist(position, check_position)
        yaw_distance = abs(get_angle_between(yaw, check_yaw))
        # Continue choosing closer positions, or in case of equal xy_distance closer orientations.
        if (
            xy_distance <= xy_tolerance
            and yaw_distance <= yaw_tolerance
            and (xy_distance < xy_tolerance or yaw_distance < min_yaw_distance)
        ):
            pose_name = check_name
            xy_tolerance = xy_distance
            # Note: A closer position has precedence and will be chosen regardless of orientation.
            min_yaw_distance = yaw_distance
    return pose_name


def find_robot_namespaces() -> List[str]:
    """Return list of robot namespaces by searching published topics for /move_base/goal."""
    _init_node()
    try:
        topics = rospy.get_published_topics()
    except ConnectionRefusedError as e:
        raise Excepthook.expect(e)

    robot_namespaces: List[str] = []
    for topic, _ in topics:
        match_result = re.match(r"([\w\/]*)\/move_base\/goal", topic)
        if match_result:
            robot_namespaces.append(match_result.group(1))
    return robot_namespaces


def add_waypoint(name: str, pose: Tuple[Sequence[float], Sequence[float]]) -> None:
    """Store waypoint with name and pose ((x, y, z), (qx, qy, qz, qw))."""
    _init_node()
    if name in Storage.waypoints.keys():
        rospy.logwarn(f"Overwriting waypoint: {Storage.waypoints[name]}")
    Storage.waypoints[name] = TuplePose.from_sequence_tuple(pose)


def save_waypoints(filepath: str = "~/.ros/robot_api_waypoints.yaml") -> None:
    """Save waypoints to file, default: '~/.ros/robot_api_waypoints.yaml'."""
    _init_node()
    if not Storage.waypoints:
        rospy.logwarn("No waypoints to save.")
        return

    filepath = os.path.expanduser(filepath)
    try:
        with open(filepath, "w") as text_file:
            for waypoint_name, waypoint in Storage.waypoints.items():
                # Note: Write tuple as list for nicer format.
                text_file.write(f"'{waypoint_name}': {TuplePose.to_str(waypoint)}\n")
    except Exception:
        rospy.logerr(f"Error while writing to file '{filepath}'!")


def load_waypoints(filepath: str = "~/.ros/robot_api_waypoints.yaml") -> None:
    """Load waypoints from file, default: '~/.ros/robot_api_waypoints.yaml'."""
    _init_node()
    filepath = os.path.expanduser(filepath)
    try:
        with open(filepath, "r") as text_file:
            lines = [line.strip() for line in text_file.readlines() if line.strip()]
        # Parse and store line by line to preserve the order within the dict read.
        for line in lines:
            elements = yaml.safe_load(line)
            assert isinstance(elements, dict), f"Invalid format in line: {line}"
            for name, pose in elements.items():
                add_waypoint(name, pose)
        rospy.loginfo(
            f"{_s(len(lines), 'waypoint')} loaded, now {len(Storage.waypoints)} in total."
        )
    except Exception:
        rospy.logerr(f"Error while reading from file '{filepath}'!")


def print_waypoints() -> None:
    """Output currently stored waypoints with rospy.loginfo()."""
    _init_node()
    rospy.loginfo(f"Available waypoints:\n" + Storage._waypoints_to_str())
