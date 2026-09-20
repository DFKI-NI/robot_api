from __future__ import annotations
from typing import Any, Callable, Mapping, Optional, Sequence, Tuple, Union, overload
import os
import time
from geometry_msgs.msg import Pose
from robot_api.extensions import Arm, Gripper
from robot_api.excepthook import Excepthook
from robot_api.lib import (
    Storage,
    TuplePose,
    get_at,
    get_pose_name,
)
from robot_api.ros_wrapper import get_ros_wrapper, RosWrapperInterface

try:
    ros_version = os.environ["ROS_VERSION"]
    if ros_version == "1":
        from tf import LookupException, ExtrapolationException
        from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
    elif ros_version == "2":
        from tf2_ros import LookupException, ExtrapolationException
        from nav2_msgs.action import NavigateToPose as MoveBaseAction

        MoveBaseGoal = MoveBaseAction.Goal
    else:
        raise ImportError(f"Unsupported ROS_VERSION: {ros_version}")
except KeyError:
    raise ImportError(
        "ROS_VERSION environment variable not set. Please source your ROS setup file."
    )


_ros_wrapper: RosWrapperInterface = get_ros_wrapper()


class Base:
    """Representation of a robot's base with navigation capabilities."""

    # Note: Cannot use move_base goal tolerances because movement by move_base does not
    #  guarantee its thresholds.
    XY_TOLERANCE = 0.2
    YAW_TOLERANCE = 0.1

    def __init__(self, namespace: str, connect_navigation_on_init: bool) -> None:
        self._namespace = namespace
        _ros_wrapper.init_action_server(
            namespace,
            {_ros_wrapper.get_move_base_topic_name(): (MoveBaseAction,)},
            connect_navigation_on_init,
        )

    def _robot_frame_id(self, robot_frame: str) -> str:
        # tf2 rejects frame ids with a leading slash, tf (ROS 1) tolerates both.
        return (self._namespace + robot_frame).lstrip("/")

    def get_pose(
        self,
        reference_frame: str = "map",
        robot_frame: str = "base_footprint",
        timeout: float = 1.0,
    ) -> Tuple[Sequence[float], Sequence[float]]:
        """Return robot pose as tuple of position [x, y, z] and orientation [x, y, z, w]."""
        try:
            pose = _ros_wrapper.lookup_transform(
                reference_frame, self._robot_frame_id(robot_frame), 0
            )
        except (LookupException, ExtrapolationException) as e:
            # If timeout is given, repeatedly try again.
            if timeout:
                time_start = time.time()
                while time.time() - time_start < timeout:
                    try:
                        time.sleep(1.0)
                        pose = _ros_wrapper.lookup_transform(
                            reference_frame,
                            self._robot_frame_id(robot_frame),
                            0,
                        )
                        return pose
                    except LookupException:
                        pass
            raise Excepthook.expect(e)

        # Note: Return results as lists because user might want to reuse and modify them.
        return pose

    def get_2d_pose(
        self,
        reference_frame: str = "map",
        robot_frame: str = "base_footprint",
        timeout: float = 1.0,
    ) -> Tuple[float, float, float]:
        """Return robot pose as (x, y, yaw in radians)."""
        position, orientation = self.get_pose(reference_frame, robot_frame, timeout)
        _, _, yaw = _ros_wrapper.euler_from_quaternion(orientation)
        return position[0], position[1], yaw

    def get_pose_name(
        self,
        poses: Mapping[
            str, Tuple[Sequence[float], Sequence[float]]
        ] = Storage.waypoints,
        xy_tolerance=XY_TOLERANCE,
        yaw_tolerance=YAW_TOLERANCE,
        timeout: float = 1.0,
    ) -> Optional[str]:
        """
        Return the name of the pose in poses closest to the robot base within the given
         tolerances.
        """
        if not poses:
            _ros_wrapper.log("No poses given to compare to.", level="warn")
            return None

        return get_pose_name(
            self.get_pose(timeout=timeout), poses, xy_tolerance, yaw_tolerance
        )

    def move_to_goal(
        self,
        goal: MoveBaseGoal,
        timeout: float = 60.0,
        done_cb: Optional[Callable[[int, Any], Any]] = None,
    ) -> Any:
        """
        Move robot to goal with timeout. Return the move_base action server's result.
        If done_cb is given, make this an asynchronous action and call done_cb() when done.
        """
        if not _ros_wrapper._connect_to_action_server(
            _ros_wrapper.get_move_base_topic_name()
        ):
            _ros_wrapper.log("Did you launch the move_base node?", level="error")
            return

        pose = TuplePose.from_pose(_ros_wrapper.get_pose_from_goal(goal))
        # Add waypoint if new, and move to goal.
        is_new_goal = pose not in Storage.waypoints.values()
        custom_goal_name = Storage._get_custom_waypoint_name(pose)
        _ros_wrapper.log(
            f"Sending {'new ' if is_new_goal else ''}navigation goal "
            + (f"'{custom_goal_name}' " if custom_goal_name else "")
            + f"{pose} ...",
            level="debug",
        )
        if is_new_goal:
            Storage._add_generic_waypoint(pose)
        if done_cb is None:
            _ros_wrapper.log(
                f"Waiting for navigation result with timeout of {timeout} s ...",
                level="debug",
            )
            return _ros_wrapper.send_goal_and_wait(
                _ros_wrapper.get_move_base_topic_name(), goal, timeout
            )
        else:
            return _ros_wrapper.send_goal(
                _ros_wrapper.get_move_base_topic_name(), goal, done_cb
            )

    def move_to_pose(
        self,
        pose: Pose,
        frame_id: str = "map",
        timeout: float = 60.0,
        done_cb: Optional[Callable[[int, Any], Any]] = None,
    ) -> Any:
        """
        Move robot to pose in frame_id's map with timeout.
         Return the move_base action server's result.
        If done_cb is given, make this an asynchronous action and call done_cb() when done.
        """
        goal = _ros_wrapper.create_move_base_goal(pose, frame_id)
        return self.move_to_goal(goal, timeout, done_cb)

    def move_to_tuple_pose(
        self,
        pose: Tuple[Sequence[float], Sequence[float]],
        frame_id: str = "map",
        timeout: float = 60.0,
        done_cb: Optional[Callable[[int, Any], Any]] = None,
    ) -> Any:
        """
        Move robot to pose in frame_id's map with timeout.
         Return the move_base action server's result.
        If done_cb is given, make this an asynchronous action and call done_cb() when done.
        """
        return self.move_to_pose(TuplePose.to_pose(pose), frame_id, timeout, done_cb)

    def move_to_position_and_orientation(
        self,
        position: Sequence[float],
        orientation: Sequence[float],
        frame_id: str = "map",
        timeout: float = 60.0,
        done_cb: Optional[Callable[[int, Any], Any]] = None,
    ) -> Any:
        """
        Move robot to position and orientation in frame_id's map with timeout.
         Return the move_base action server's result.
        If done_cb is given, make this an asynchronous action and call done_cb() when done.
        """
        return self.move_to_pose(
            TuplePose.to_pose((position, orientation)), frame_id, timeout, done_cb
        )

    def move_to_coordinates(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        frame_id: str = "map",
        timeout: float = 60.0,
        done_cb: Optional[Callable[[int, Any], Any]] = None,
    ) -> Any:
        """
        Move robot to given 6D pose in frame_id's map with timeout.
         Return the move_base action server's result.
        If done_cb is given, make this an asynchronous action and call done_cb() when done.
        """
        return self.move_to_pose(
            TuplePose.to_pose(
                ((x, y, z), _ros_wrapper.quaternion_from_euler(roll, pitch, yaw))
            ),
            frame_id,
            timeout,
            done_cb,
        )

    MOVE_LINEAR_TOPIC_NAME = "move_linear"

    def move_linear(self, distance_cm: float, timeout: float = 60.0) -> Any:
        """
        Drive the base straight along its current heading by distance_cm (negative = backward)
        through a MoveLinear action server (mobipick_base_motion; it plans the shifted pose with
        move_base and falls back to a guarded straight drive). Return the server's result
        (success, message, travelled_cm, lateral_error_cm, heading_error_deg) or None when the
        server is unavailable or the timeout elapsed.
        """
        try:
            from mobipick_base_motion.msg import MoveLinearAction, MoveLinearGoal
        except ImportError:
            _ros_wrapper.log(
                "move_linear needs the mobipick_base_motion package (MoveLinear action, ROS 1).",
                level="error",
            )
            return None
        server_name = self.MOVE_LINEAR_TOPIC_NAME
        _ros_wrapper._server_specs.setdefault(server_name, (MoveLinearAction,))
        if not _ros_wrapper._connect_to_action_server(server_name, timeout=2.0):
            _ros_wrapper.log("Did you launch the move_linear node?", level="error")
            return None
        _ros_wrapper.log(f"Moving base {distance_cm:+.1f} cm along its heading ...", level="info")
        state = _ros_wrapper.send_goal_and_wait(
            server_name,
            MoveLinearGoal(distance_cm=float(distance_cm), timeout_s=float(timeout)),
            timeout + 5.0,
        )
        result = _ros_wrapper.get_action_result(server_name)
        if result is None:
            _ros_wrapper.log(f"No move_linear result (state {state}).", level="error")
            return None
        _ros_wrapper.log(f"move_linear: {result.message}", level="info" if result.success else "error")
        return result

    def move_to_waypoint(
        self,
        name: str,
        frame_id: str = "map",
        timeout: float = 60.0,
        done_cb: Optional[Callable[[int, Any], Any]] = None,
    ) -> Any:
        """
        Move robot to waypoint by name in frame_id's map with timeout.
         Return the move_base action server's result.
        If done_cb is given, make this an asynchronous action and call done_cb() when done.
        """
        if name not in Storage.waypoints.keys():
            if Storage.waypoints:
                _ros_wrapper.log(
                    f"Waypoint '{name}' does not exist. Available waypoints:\n"
                    + Storage._waypoints_to_str(),
                    level="error",
                )
            else:
                _ros_wrapper.log(
                    f"No waypoints defined yet, so cannot use waypoint '{name}'.",
                    level="error",
                )
            return

        return self.move_to_pose(
            TuplePose.to_pose(Storage.waypoints[name]), frame_id, timeout, done_cb
        )

    @overload
    def move(
        self,
        goal: MoveBaseGoal,
        frame_id: str = "map",
        timeout: float = 60.0,
        done_cb: Optional[Callable[[int, Any], Any]] = None,
    ) -> Any:
        """
        Move robot to goal with timeout. Return the move_base action server's result.
        If done_cb is given, make this an asynchronous action and call done_cb() when done.
        """
        ...

    @overload
    def move(
        self,
        pose: Union[Pose, Tuple[Sequence[float], Sequence[float]]],
        frame_id: str = "map",
        timeout: float = 60.0,
        done_cb: Optional[Callable[[int, Any], Any]] = None,
    ) -> Any:
        """
        Move robot to pose in frame_id's map with timeout.
         Return the move_base action server's result.
        If done_cb is given, make this an asynchronous action and call done_cb() when done.
        """
        ...

    @overload
    def move(
        self,
        position: Sequence[float],
        orientation: Sequence[float],
        frame_id: str = "map",
        timeout: float = 60.0,
        done_cb: Optional[Callable[[int, Any], Any]] = None,
    ) -> Any:
        """
        Move robot to position and orientation in frame_id's map with timeout.
         Return the move_base action server's result.
        If done_cb is given, make this an asynchronous action and call done_cb() when done.
        """
        ...

    @overload
    def move(
        self,
        x: float,
        y: float,
        yaw: float,
        frame_id: str = "map",
        timeout: float = 60.0,
        done_cb: Optional[Callable[[int, Any], Any]] = None,
    ) -> Any:
        """
        Move robot to pose given by x, y, and yaw in radians.
         Return the move_base action server's result.
        If done_cb is given, make this an asynchronous action and call done_cb() when done.
        """
        ...

    @overload
    def move(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        frame_id: str = "map",
        timeout: float = 60.0,
        done_cb: Optional[Callable[[int, Any], Any]] = None,
    ) -> Any:
        """
        Move robot to given 6D pose in frame_id's map with timeout.
         Return the move_base action server's result.
        If done_cb is given, make this an asynchronous action and call done_cb() when done.
        """
        ...

    def move(
        self,
        *args: Any,
        frame_id: str = "map",
        timeout: float = 60.0,
        done_cb: Optional[Callable[[int, Any], Any]] = None,
        **kwargs: Any,
    ) -> Any:
        goal: Optional[MoveBaseGoal] = kwargs.get("goal", get_at(args, 0, MoveBaseGoal))
        pose: Optional[
            Union[Pose, Tuple[Sequence[Union[float, int]], Sequence[Union[float, int]]]]
        ] = kwargs.get(
            "pose",
            get_at(
                args,
                0,
                Union[
                    Pose,
                    Tuple[Sequence[Union[float, int]], Sequence[Union[float, int]]],
                ],
            ),
        )
        position: Optional[Sequence[Union[float, int]]] = kwargs.get(
            "position", get_at(args, 0, Sequence[Union[float, int]])
        )
        orientation: Optional[Sequence[Union[float, int]]] = kwargs.get(
            "orientation", get_at(args, 1, Sequence[Union[float, int]])
        )
        x: Union[float, int] = kwargs.get("x", get_at(args, 0, Union[float, int]))
        y: Union[float, int] = kwargs.get("y", get_at(args, 1, Union[float, int]))
        z: Union[float, int] = kwargs.get(
            "z", get_at(args, 2, Union[float, int]) if len(args) > 3 else 0.0
        )
        roll: Union[float, int] = kwargs.get(
            "roll", get_at(args, 3, Union[float, int]) if len(args) >= 4 else 0.0
        )
        pitch: Union[float, int] = kwargs.get(
            "pitch", get_at(args, 4, Union[float, int]) if len(args) >= 5 else 0.0
        )
        yaw: Union[float, int] = kwargs.get(
            "yaw", get_at(args, 5 if len(args) >= 6 else 2, Union[float, int])
        )
        if goal is None:
            if pose is None:
                if not position or not orientation:
                    if not position and not orientation:
                        if x is None or y is None or yaw is None:
                            raise Excepthook.expect(
                                ValueError(
                                    "1. goal, 2. pose, 3. position and orientation,"
                                    " or 4. x, y, and yaw must be specified."
                                )
                            )
                        return self.move_to_coordinates(
                            x, y, z, roll, pitch, yaw, frame_id, timeout, done_cb
                        )
                    else:
                        # Raise error if only one of position and orientation is specified.
                        raise Excepthook.expect(
                            ValueError(
                                "Both 'position' and 'orientation' parameters"
                                " must be specified."
                            )
                        )
                return self.move_to_position_and_orientation(
                    position, orientation, frame_id, timeout, done_cb
                )
            return (
                self.move_to_pose(pose, frame_id, timeout, done_cb)
                if isinstance(pose, Pose)
                else self.move_to_tuple_pose(pose, frame_id, timeout, done_cb)
            )
        return self.move_to_goal(goal, timeout, done_cb)


class Robot:
    def __init__(
        self,
        namespace: str = "/",
        connect_navigation_on_init: bool = False,
        connect_manipulation_on_init: bool = False,
    ) -> None:
        _ros_wrapper._init_node()
        # Make sure namespace naming is correct.
        if not namespace.startswith("/"):
            namespace = "/" + namespace
        if not namespace.endswith("/"):
            namespace += "/"
        self.namespace = namespace
        self.base = Base(namespace, connect_navigation_on_init)
        self.arm = Arm(namespace, connect_manipulation_on_init)
        self.gripper = Gripper(namespace, connect_manipulation_on_init)
