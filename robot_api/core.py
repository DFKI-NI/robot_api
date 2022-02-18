#!/usr/bin/env python3
from __future__ import annotations
from typing import Any, Callable, Mapping, Optional, Sequence, Tuple, Type, Union, overload
import time
import math
import rospy
import tf
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from robot_api.extensions import Arm
from robot_api.excepthook import Excepthook
from robot_api.lib import Action, ActionlibComponent, Storage, TuplePose, get_angle_between, _init_node


def _isinstance(obj: object, type_or_types: Union[Type, Tuple[Type, ...]]) -> bool:
    """Return whether obj's and its potential elements' types match type_or_types."""
    if isinstance(type_or_types, tuple):
        return _isinstance(obj, type_or_types[0]) and (len(type_or_types) <= 1 \
            or isinstance(obj, Sequence) and all(_isinstance(element, type_or_types[1:]) for element in obj))

    return isinstance(obj, type_or_types)


def _get_at(args: Any, index: int, type_or_types: Union[Type, Tuple[Type, ...]]) -> Any:
    """Return element in args at index if its type matches type_or_types, else None."""
    return args[index] if isinstance(args, Sequence) and len(args) > index \
        and _isinstance(args[index], type_or_types) else None


class BaseMoveToGoalAction(Action):
    @staticmethod
    def execute(base: Base, goal: MoveBaseGoal, timeout: float=60.0,
            done_cb: Optional[Callable[[int, MoveBaseResult], Any]]=None) -> Any:
        if not base._connect(Base.MOVE_BASE_TOPIC_NAME):
            return

        pose = TuplePose.from_pose(goal.target_pose.pose)
        # Add waypoint if new, and move to goal.
        is_new_goal = pose not in Storage.waypoints.values()
        custom_goal_name = Storage._get_custom_waypoint_name(pose)
        rospy.logdebug(f"Sending {'new ' if is_new_goal else ''}navigation goal " \
            + (f"'{custom_goal_name}' " if custom_goal_name else "") + f"{pose} ...")
        if is_new_goal:
            Storage._add_generic_waypoint(pose)
        if done_cb is None:
            rospy.logdebug(f"Waiting for navigation result with timeout of {timeout} s ...")
            return base._action_clients[Base.MOVE_BASE_TOPIC_NAME].send_goal_and_wait(goal, rospy.Duration(timeout))
        else:
            return base._action_clients[Base.MOVE_BASE_TOPIC_NAME].send_goal(goal, done_cb)


class BaseMoveToPoseAction(Action):
    @staticmethod
    def execute(base: Base, pose: Pose, frame_id: str="map", timeout: float=60.0,
            done_cb: Optional[Callable[[int, MoveBaseResult], Any]]=None) -> Any:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        return BaseMoveToGoalAction.execute(base, goal, timeout, done_cb)


class BaseMoveToTuplePoseAction(Action):
    @staticmethod
    def execute(base: Base, pose: Tuple[Sequence[float], Sequence[float]], frame_id: str="map", timeout: float=60.0,
            done_cb: Optional[Callable[[int, MoveBaseResult], Any]]=None) -> Any:
        return BaseMoveToPoseAction.execute(base, TuplePose.to_pose(pose), frame_id, timeout, done_cb)


class BaseMoveToPositionOrientationAction(Action):
    @staticmethod
    def execute(base: Base, position: Sequence[float], orientation: Sequence[float], frame_id: str="map",
            timeout: float=60.0, done_cb: Optional[Callable[[int, MoveBaseResult], Any]]=None) -> Any:
        return BaseMoveToPoseAction.execute(base, TuplePose.to_pose((position, orientation)),
            frame_id, timeout, done_cb)


class BaseMoveToCoordinatesAction(Action):
    @staticmethod
    def execute(base: Base, x: float, y: float, z: float, roll: float, pitch: float, yaw: float,
            frame_id: str="map", timeout: float=60.0,
            done_cb: Optional[Callable[[int, MoveBaseResult], Any]]=None) -> Any:
        return BaseMoveToPoseAction.execute(base, TuplePose.to_pose(((x, y, z),
            tf.transformations.quaternion_from_euler(roll, pitch, yaw))), frame_id, timeout, done_cb)


class Base(ActionlibComponent):
    MOVE_BASE_TOPIC_NAME = "move_base"
    # Note: Cannot use move_base goal tolerances because movement by move_base does not guarantee its thresholds.
    XY_TOLERANCE = 0.2
    YAW_TOLERANCE = 0.1

    def __init__(self, namespace: str, connect_navigation_on_init: bool) -> None:
        super().__init__(namespace, {self.MOVE_BASE_TOPIC_NAME: (MoveBaseAction,)}, connect_navigation_on_init)
        self._tf_listener = tf.TransformListener()

    def get_pose(self, reference_frame: str="map", robot_frame: str="base_footprint",
            timeout: float=1.0) -> Tuple[Sequence[float], Sequence[float]]:
        """Return robot pose as tuple of position [x, y, z] and orientation [x, y, z, w]."""
        try:
            pose = self._tf_listener.lookupTransform(reference_frame, self._namespace + robot_frame, rospy.Time(0))
        except (tf.LookupException, tf.ExtrapolationException) as e:
            # If timeout is given, repeatedly try again.
            if timeout:
                time_start = time.time()
                while time.time() - time_start < timeout:
                    try:
                        time.sleep(1.0)
                        pose = self._tf_listener.lookupTransform(reference_frame,
                            self._namespace + robot_frame, rospy.Time(0))
                        return pose
                    except tf.LookupException:
                        pass
            raise Excepthook.expect(e)

        # Note: Return results as lists because user might want to reuse and modify them.
        return pose

    def get_2d_pose(self, reference_frame: str="map", robot_frame: str="base_footprint",
            timeout: float=1.0) -> Tuple[float, float, float]:
        """Return robot pose as (x, y, yaw in radians)."""
        position, orientation = self.get_pose(reference_frame, robot_frame, timeout)
        _, _, yaw = tf.transformations.euler_from_quaternion(orientation)
        return position[0], position[1], yaw

    def get_pose_name(self, poses: Mapping[str, Tuple[Sequence[float], Sequence[float]]]=Storage.waypoints,
            xy_tolerance=XY_TOLERANCE, yaw_tolerance=YAW_TOLERANCE, timeout: float=1.0) -> Optional[str]:
        """Return the name of the pose in poses closest to the robot base within the given tolerances."""
        if not poses:
            rospy.logwarn("No poses given to compare to.")
            return None

        position, orientation = self.get_pose(timeout=timeout)
        _, _, yaw = tf.transformations.euler_from_quaternion(orientation)
        pose_name: Optional[str] = None
        min_yaw_distance = math.pi
        for check_name, (check_position, check_orientation) in poses.items():
            _, _, check_yaw = tf.transformations.euler_from_quaternion(check_orientation)
            xy_distance = math.dist(position, check_position)
            yaw_distance = abs(get_angle_between(yaw, check_yaw))
            # Continue choosing closer positions, or in case of equal xy_distance closer orientations.
            if xy_distance <= xy_tolerance and yaw_distance <= yaw_tolerance \
                    and (xy_distance < xy_tolerance or yaw_distance < min_yaw_distance):
                pose_name = check_name
                xy_tolerance = xy_distance
                # Note: A closer position has precedence and will be chosen regardless of orientation.
                min_yaw_distance = yaw_distance
        return pose_name

    def move_to_waypoint(self, name: str, frame_id: str="map", timeout: float=60.0,
            done_cb: Optional[Callable[[int, MoveBaseResult], Any]]=None) -> Any:
        """Move robot to waypoint by name in frame_id's map with timeout.
        Optionally, call done_cb() afterwards if given.
        """
        if name not in Storage.waypoints.keys():
            if Storage.waypoints:
                rospy.logerr(f"Waypoint '{name}' does not exist. Available waypoints:\n"
                    + Storage._waypoints_to_str())
            else:
                rospy.logerr(f"No waypoints defined yet, so cannot use waypoint '{name}'.")
            return

        return BaseMoveToPoseAction.execute(self, TuplePose.to_pose(Storage.waypoints[name]),
            frame_id, timeout, done_cb)

    @overload
    def move(self, goal: MoveBaseGoal, frame_id: str="map", timeout: float=60.0,
            done_cb: Optional[Callable[[int, MoveBaseResult], Any]]=None) -> Any:
        """Move robot to goal. Optionally, call done_cb() afterwards if given."""
        ...

    @overload
    def move(self, pose: Union[Pose, Tuple[Sequence[float], Sequence[float]]], frame_id: str="map",
            timeout: float=60.0, done_cb: Optional[Callable[[int, MoveBaseResult], Any]]=None) -> Any:
        """Move robot to pose. Optionally, call done_cb() afterwards if given."""
        ...

    @overload
    def move(self, position: Sequence[float], orientation: Sequence[float], frame_id: str="map", timeout: float=60.0,
            done_cb: Optional[Callable[[int, MoveBaseResult], Any]]=None) -> Any:
        """Move robot to position and orientation. Optionally, call done_cb() afterwards if given."""
        ...

    @overload
    def move(self, x: float, y: float, yaw: float, frame_id: str="map", timeout: float=60.0,
            done_cb: Optional[Callable[[int, MoveBaseResult], Any]]=None) -> Any:
        """Move robot to pose given by x, y, and yaw in radians. Optionally, call done_cb() afterwards if given."""
        ...

    @overload
    def move(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float, frame_id: str="map",
            timeout: float=60.0, done_cb: Optional[Callable[[int, MoveBaseResult], Any]]=None) -> Any:
        """Move robot to given 6D pose. Optionally, call done_cb() afterwards if given."""
        ...

    def move(self, *args: Any, frame_id: str="map", timeout: float=60.0,
            done_cb: Optional[Callable[[int, MoveBaseResult], Any]]=None, **kwargs: Any) -> Any:
        goal: Optional[MoveBaseGoal] = kwargs.get("goal", _get_at(args, 0, MoveBaseGoal))
        pose: Optional[Union[Pose, Tuple[Sequence[float], Sequence[float]]]] = kwargs.get("pose",
            _get_at(args, 0, Pose) or _get_at(args, 0, (tuple, Sequence, float)))
        position: Optional[Sequence[float]] = kwargs.get("position", _get_at(args, 0, (Sequence, float)))
        orientation: Optional[Sequence[float]] = kwargs.get("orientation", _get_at(args, 1, (Sequence, float)))
        x: float = kwargs.get("x", _get_at(args, 0, float))
        y: float = kwargs.get("y", _get_at(args, 1, float))
        z: float = kwargs.get("z", _get_at(args, 2, float) if len(args) > 3 else 0.0)
        roll: float = kwargs.get("roll", _get_at(args, 3, float) if len(args) >= 4 else 0.0)
        pitch: float = kwargs.get("pitch", _get_at(args, 4, float) if len(args) >= 5 else 0.0)
        yaw: float = kwargs.get("yaw", _get_at(args, 5 if len(args) >= 6 else 2, float))
        if goal is None:
            if pose is None:
                if not position or not orientation:
                    if not position and not orientation:
                        if x is None or y is None or yaw is None:
                            raise Excepthook.expect(ValueError("1. Goal, 2. pose, 3. position and orientation,"
                                " or 4. x, y, and yaw must be specified."))
                        return BaseMoveToCoordinatesAction.execute(self, x, y, z, roll, pitch, yaw, frame_id, timeout,
                            done_cb)
                    else:
                        # Raise error if only one of position and orientation is specified.
                        raise Excepthook.expect(ValueError("Both 'position' and 'orientation' parameters"
                            " must be specified."))
                return BaseMoveToPositionOrientationAction.execute(self, position, orientation, frame_id, timeout,
                    done_cb)
            return BaseMoveToPoseAction.execute(self, pose, frame_id, timeout, done_cb) if isinstance(pose, Pose) \
                else BaseMoveToTuplePoseAction.execute(self, pose, frame_id, timeout, done_cb)
        return BaseMoveToGoalAction.execute(self, goal, timeout, done_cb)


class Robot:
    def __init__(self, namespace: str=rospy.get_namespace(), connect_navigation_on_init: bool=False,
            connect_manipulation_on_init: bool=False) -> None:
        _init_node()
        # Make sure namespace naming is correct.
        if not namespace.startswith('/'):
            namespace = '/' + namespace
        if not namespace.endswith('/'):
            namespace += '/'
        self.namespace = namespace
        self.base = Base(namespace, connect_navigation_on_init)
        self.arm = Arm(namespace, connect_manipulation_on_init)
