from __future__ import annotations
from typing import Any, Callable, Mapping, Optional, Sequence, Tuple, Union, overload
import time
import rospy
import tf
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from robot_api.extensions import Arm, Gripper
from robot_api.excepthook import Excepthook
from robot_api.lib import (
    ActionlibComponent,
    Storage,
    TuplePose,
    _init_node,
    get_at,
    get_pose_name,
)


class Base(ActionlibComponent):
    """Representation of a robot's base with navigation capabilities."""

    MOVE_BASE_TOPIC_NAME = "move_base"
    # Note: Cannot use move_base goal tolerances because movement by move_base does not
    #  guarantee its thresholds.
    XY_TOLERANCE = 0.2
    YAW_TOLERANCE = 0.1

    def __init__(self, namespace: str, connect_navigation_on_init: bool) -> None:
        super().__init__(
            namespace,
            {self.MOVE_BASE_TOPIC_NAME: (MoveBaseAction,)},
            connect_navigation_on_init,
        )
        self._tf_listener = tf.TransformListener()

    def get_pose(
        self,
        reference_frame: str = "map",
        robot_frame: str = "base_footprint",
        timeout: float = 1.0,
    ) -> Tuple[Sequence[float], Sequence[float]]:
        """Return robot pose as tuple of position [x, y, z] and orientation [x, y, z, w]."""
        try:
            pose = self._tf_listener.lookupTransform(
                reference_frame, self._namespace + robot_frame, rospy.Time(0)
            )
        except (tf.LookupException, tf.ExtrapolationException) as e:
            # If timeout is given, repeatedly try again.
            if timeout:
                time_start = time.time()
                while time.time() - time_start < timeout:
                    try:
                        time.sleep(1.0)
                        pose = self._tf_listener.lookupTransform(
                            reference_frame,
                            self._namespace + robot_frame,
                            rospy.Time(0),
                        )
                        return pose
                    except tf.LookupException:
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
        _, _, yaw = tf.transformations.euler_from_quaternion(orientation)
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
            rospy.logwarn("No poses given to compare to.")
            return None

        return get_pose_name(
            self.get_pose(timeout=timeout), poses, xy_tolerance, yaw_tolerance
        )

    def move_to_goal(
        self,
        goal: MoveBaseGoal,
        timeout: float = 60.0,
        done_cb: Optional[Callable[[int, MoveBaseResult], Any]] = None,
    ) -> Any:
        """
        Move robot to goal with timeout. Return the move_base action server's result.
        If done_cb is given, make this an asynchronous action and call done_cb() when done.
        """
        if not self._connect(Base.MOVE_BASE_TOPIC_NAME):
            rospy.logerr("Did you launch the move_base node?")
            return

        pose = TuplePose.from_pose(goal.target_pose.pose)
        # Add waypoint if new, and move to goal.
        is_new_goal = pose not in Storage.waypoints.values()
        custom_goal_name = Storage._get_custom_waypoint_name(pose)
        rospy.logdebug(
            f"Sending {'new ' if is_new_goal else ''}navigation goal "
            + (f"'{custom_goal_name}' " if custom_goal_name else "")
            + f"{pose} ..."
        )
        if is_new_goal:
            Storage._add_generic_waypoint(pose)
        if done_cb is None:
            rospy.logdebug(
                f"Waiting for navigation result with timeout of {timeout} s ..."
            )
            return self._action_clients[Base.MOVE_BASE_TOPIC_NAME].send_goal_and_wait(
                goal, rospy.Duration(timeout)
            )
        else:
            return self._action_clients[Base.MOVE_BASE_TOPIC_NAME].send_goal(
                goal, done_cb
            )

    def move_to_pose(
        self,
        pose: Pose,
        frame_id: str = "map",
        timeout: float = 60.0,
        done_cb: Optional[Callable[[int, MoveBaseResult], Any]] = None,
    ) -> Any:
        """
        Move robot to pose in frame_id's map with timeout.
         Return the move_base action server's result.
        If done_cb is given, make this an asynchronous action and call done_cb() when done.
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        return self.move_to_goal(goal, timeout, done_cb)

    def move_to_tuple_pose(
        self,
        pose: Tuple[Sequence[float], Sequence[float]],
        frame_id: str = "map",
        timeout: float = 60.0,
        done_cb: Optional[Callable[[int, MoveBaseResult], Any]] = None,
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
        done_cb: Optional[Callable[[int, MoveBaseResult], Any]] = None,
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
        done_cb: Optional[Callable[[int, MoveBaseResult], Any]] = None,
    ) -> Any:
        """
        Move robot to given 6D pose in frame_id's map with timeout.
         Return the move_base action server's result.
        If done_cb is given, make this an asynchronous action and call done_cb() when done.
        """
        return self.move_to_pose(
            TuplePose.to_pose(
                ((x, y, z), tf.transformations.quaternion_from_euler(roll, pitch, yaw))
            ),
            frame_id,
            timeout,
            done_cb,
        )

    def move_to_waypoint(
        self,
        name: str,
        frame_id: str = "map",
        timeout: float = 60.0,
        done_cb: Optional[Callable[[int, MoveBaseResult], Any]] = None,
    ) -> Any:
        """
        Move robot to waypoint by name in frame_id's map with timeout.
         Return the move_base action server's result.
        If done_cb is given, make this an asynchronous action and call done_cb() when done.
        """
        if name not in Storage.waypoints.keys():
            if Storage.waypoints:
                rospy.logerr(
                    f"Waypoint '{name}' does not exist. Available waypoints:\n"
                    + Storage._waypoints_to_str()
                )
            else:
                rospy.logerr(
                    f"No waypoints defined yet, so cannot use waypoint '{name}'."
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
        done_cb: Optional[Callable[[int, MoveBaseResult], Any]] = None,
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
        done_cb: Optional[Callable[[int, MoveBaseResult], Any]] = None,
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
        done_cb: Optional[Callable[[int, MoveBaseResult], Any]] = None,
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
        done_cb: Optional[Callable[[int, MoveBaseResult], Any]] = None,
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
        done_cb: Optional[Callable[[int, MoveBaseResult], Any]] = None,
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
        done_cb: Optional[Callable[[int, MoveBaseResult], Any]] = None,
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
        namespace: str = rospy.get_namespace(),
        connect_navigation_on_init: bool = False,
        connect_manipulation_on_init: bool = False,
    ) -> None:
        _init_node()
        # Make sure namespace naming is correct.
        if not namespace.startswith("/"):
            namespace = "/" + namespace
        if not namespace.endswith("/"):
            namespace += "/"
        self.namespace = namespace
        self.base = Base(namespace, connect_navigation_on_init)
        self.arm = Arm(namespace, connect_manipulation_on_init)
        self.gripper = Gripper(namespace, connect_manipulation_on_init)
