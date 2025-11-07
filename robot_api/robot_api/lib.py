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
import math
import re
import yaml
from collections import OrderedDict

from geometry_msgs.msg import Point, Pose, Quaternion
from robot_api.excepthook import Excepthook
from robot_api.ros_wrapper import get_ros_wrapper


_ros_wrapper = get_ros_wrapper()

def _s(count: int, name: str, plural: str = "s") -> str:
    """Return name with or without plural ending depending on count."""
    return f"{count} {name}{plural if count != 1 else ''}"

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
        return Pose(position=Point(x=pose[0][0], y=pose[0][1], z=pose[0][2]),
                    orientation=Quaternion(x=pose[1][0], y=pose[1][1], z=pose[1][2], w=pose[1][3]))

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

def get_pose_name(pose: Tuple[Sequence[float], Sequence[float]],
                  poses: Mapping = Storage.waypoints,
                  xy_tolerance=math.inf,
                  yaw_tolerance=math.inf
    ) -> Optional[str]:
    position, orientation = pose
    _, _, yaw = _ros_wrapper.euler_from_quaternion(orientation)
    pose_name: Optional[str] = None
    min_yaw_distance = math.pi
    for check_name, (check_position, check_orientation) in poses.items():
        _, _, check_yaw = _ros_wrapper.euler_from_quaternion(check_orientation)
        xy_distance = math.dist(position, check_position)
        yaw_distance = abs(get_angle_between(yaw, check_yaw))
        if xy_distance <= xy_tolerance and yaw_distance <= yaw_tolerance and (xy_distance < xy_tolerance or yaw_distance < min_yaw_distance):
            pose_name = check_name
            xy_tolerance = xy_distance
            min_yaw_distance = yaw_distance
    return pose_name

def find_robot_namespaces() -> List[str]:
    _ros_wrapper._init_node()
    try:
        topics = _ros_wrapper.get_topics()
    except ConnectionRefusedError as e:
        raise Excepthook.expect(e)
    robot_namespaces: List[str] = []
    for topic, _ in topics:
        if (match := re.match(r"([\w\/]*)\/move_base\/goal", topic)):
            robot_namespaces.append(match.group(1))
    return robot_namespaces

def add_waypoint(name: str, pose: Tuple[Sequence[float], Sequence[float]]) -> None:
    _ros_wrapper._init_node()
    if name in Storage.waypoints:
        _ros_wrapper.log(f"Overwriting waypoint: {Storage.waypoints[name]}", level="warn")
    Storage.waypoints[name] = TuplePose.from_sequence_tuple(pose)

def save_waypoints(filepath: str = "~/.ros/robot_api_waypoints.yaml") -> None:
    _ros_wrapper._init_node()
    if not Storage.waypoints:
        _ros_wrapper.log("No waypoints to save.", level="warn")
        return
    filepath = os.path.expanduser(filepath)
    try:
        with open(filepath, "w") as f:
            for name, waypoint in Storage.waypoints.items():
                f.write(f"'{name}': {TuplePose.to_str(waypoint)}\n")
    except Exception:
        _ros_wrapper.log(f"Error while writing to file '{filepath}'!", level="error")

def load_waypoints(filepath: str = "~/.ros/robot_api_waypoints.yaml") -> None:
    _ros_wrapper._init_node()
    filepath = os.path.expanduser(filepath)
    try:
        with open(filepath, "r") as f:
            lines = [line.strip() for line in f if line.strip()]
        for line in lines:
            elements = yaml.safe_load(line)
            assert isinstance(elements, dict), f"Invalid format in line: {line}"
            for name, pose in elements.items():
                add_waypoint(name, pose)
        _ros_wrapper.log(f"{_s(len(lines), 'waypoint')} loaded, now {len(Storage.waypoints)} in total.", level="info")
    except Exception:
        _ros_wrapper.log(f"Error while reading from file '{filepath}'!", level="error")

def print_waypoints() -> None:
    _ros_wrapper._init_node()
    _ros_wrapper.log(f"Available waypoints:\n" + Storage._waypoints_to_str(), level="info")
