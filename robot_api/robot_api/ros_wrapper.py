from __future__ import annotations
import os
import shlex
import subprocess
import time
from threading import Event
from abc import ABC, abstractmethod
from typing import Optional, List, Tuple, Any, Dict, TypeVar, Generic, Union

try:
    ros_version = os.environ["ROS_VERSION"]
    if ros_version == "1":
        import rospy
        import rosnode
        import rosgraph
        import rosparam
        import actionlib
        import tf
        from tf.transformations import euler_from_quaternion, quaternion_from_euler
        from rospy import Duration
        from move_base_msgs.msg import MoveBaseGoal
        from control_msgs.msg import GripperCommandGoal
    elif ros_version == "2":
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from rclpy.duration import Duration
        from rclpy.client import Future
        from tf_transformations import euler_from_quaternion, quaternion_from_euler
        from tf2_ros import TransformListener, Buffer
        from nav2_msgs.action import NavigateToPose
        from control_msgs.action import ParallelGripperCommand
    else:
        raise ImportError(f"Unsupported ROS_VERSION: {ros_version}")
except KeyError:
    raise ImportError("ROS_VERSION environment variable not set. Please source your ROS setup file.")


T = TypeVar("T")
class RosWrapperInterface(ABC):
    @abstractmethod
    def _init_node(self):
        pass

    @abstractmethod
    def log(self, message: str, level: str = "info", *args, **kwargs) -> None:
        pass

    @abstractmethod
    def get_topics(self, namespace: str = "") -> List[Tuple[str, List[str]]]:
        pass

    @abstractmethod
    def get_action_result(self, server_name: Optional[str] = None) -> Any:
        pass

    @abstractmethod
    def _connect_to_action_server(self, server_name: str, timeout: float = 0.0) -> bool:
        pass

    @abstractmethod
    def _has_action_result(self, server_name: str) -> bool:
        pass

    @abstractmethod
    def init_action_server(self, namespace: str,
                           server_specs: Dict[str, Union[Tuple[Generic[T]], Tuple[Generic[T], str, int]]],
                           connect_on_init: bool = False):
        pass

    @abstractmethod
    def get_action_result(self, server_name: Optional[str] = None) -> Any:
        pass

    @abstractmethod
    def lookup_transform(self, target_frame: str, source_frame: str, time: Any) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
        pass

    @abstractmethod
    def send_goal_and_wait(self, server_name: str, goal: T, timeout: float = 60.0) -> Any:
        pass

    @abstractmethod
    def send_goal(self, server_name: str, goal: T, done_cb: Any) -> None:
        pass

    @abstractmethod
    def create_move_base_goal(self, pose, frame_id: str) -> Any:
        pass

    @abstractmethod
    def create_open_gripper_goal(self, position: float, max_effort: float = 0.0) -> Any:
        pass

    @abstractmethod
    def create_close_gripper_goal(self, position: float, max_effort: float = 0.0) -> Any:
        pass

    @abstractmethod
    def get_pose_from_goal(self, goal: Any) -> Tuple[List[float], List[float]]:
        pass

    @abstractmethod
    def get_move_base_topic_name(self) -> str:
        pass

    @abstractmethod
    def get_moveit_topic_name(self) -> str:
        pass

    @abstractmethod
    def get_ft_observer_topic_name(self) -> str:
        pass

    @abstractmethod
    def get_gripper_topic_name(self) -> str:
        pass

    @abstractmethod
    def wait_for_message(self, topic: str, message_type: Any, timeout: Optional[float] = None) -> Any:
        pass

    @abstractmethod
    def list_params(self, namespace: str = "") -> List[str]:
        pass

    @abstractmethod
    def get_param(self, param_name: str) -> Any:
        pass

    @abstractmethod
    def launch_moveit_macros_command(self) -> str:
        pass

    def euler_from_quaternion(self, orientation: List[float], axes: str = "sxyz") -> Tuple[float, float, float]:
        return euler_from_quaternion(orientation, axes=axes)
    
    def quaternion_from_euler(self, roll: float, pitch: float, yaw: float, axes: str = "sxyz") -> List[float]:
        return quaternion_from_euler(roll, pitch, yaw, axes=axes)
    
    def _is_topic_with_suffix(self, namespace: str, topic: str, suffix: str) -> bool:
        for check_topic, check_message_type in self.get_topics(namespace):
            if check_topic == namespace + topic and check_message_type[0].endswith(suffix):
                return True
        return False
    
    def _is_topic_of_type(self, namespace: str, topic: str, message_type: str) -> bool:
        for check_topic, check_message_type in self.get_topics(namespace):
            if check_topic == namespace + topic and check_message_type[0].split('/')[-1] == message_type:
                return True
        return False
    
    def _execute(self, command: str, sleep_duration: int = 0) -> None:
        self.log(command, level="info")
        subprocess.Popen(shlex.split(command), stdout=subprocess.DEVNULL)
        if sleep_duration:
            time.sleep(sleep_duration)

class Ros1Wrapper(RosWrapperInterface):
    MOVE_BASE_TOPIC_NAME = "move_base"
    MOVEIT_TOPIC_NAME = "moveit_macros"
    FT_OBSERVER_TOPIC_NAME = "ft_observer"
    GRIPPER_TOPIC_NAME = "gripper_hw"
    _namespace: str = ""
    _server_specs: Dict[str, Union[Tuple[Generic[T]], Tuple[Generic[T], str, int]]] = {}
    _action_clients: Dict[str, actionlib.SimpleActionClient] = {}
    _last_server_name: Optional[str] = None
    _tf_listener: Optional[tf.TransformListener] = None

    def _init_node(self) -> None:
        if not rosgraph.is_master_online():
            print("Waiting for ROS master node to go online ...")
            while not rosgraph.is_master_online():
                time.sleep(1.0)
        if rospy.is_shutdown():
            rospy.logerr("ROS is shutting down.")
            return
        name = f"robot_api_{os.getpid()}"
        if f"/{name}" not in rosnode.get_node_names():
            try:
                rospy.init_node(name)
                rospy.logdebug(f"ROS node '{name}' initialized.")
            except rospy.ROSException:
                pass

    def log(self, message: str, level: str = "info", *args, **kwargs) -> None:
        if level == "info":
            rospy.loginfo(message, *args)
        elif level == "warn":
            rospy.logwarn(message, *args)
        elif level == "error":
            rospy.logerr(message, *args)
        elif level == "debug":
            rospy.logdebug(message, *args)
        else:
            rospy.loginfo(message, *args)

    def get_topics(self, namespace: str = "") -> List[Tuple[str, List[str]]]:
        return [(topic_name, [topic_type,]) for topic_name, topic_type in rospy.get_published_topics(namespace)]
    
    def init_action_server(self, namespace: str,
                           server_specs: Dict[str, Union[Tuple[Generic[T]], Tuple[Generic[T], str, int]]],
                           connect_on_init: bool = False):
        self._namespace = namespace
        assert server_specs, "Error: You must init an ActionlibComponent with an item in server_specs."
        self._server_specs.update(server_specs)
        self._action_clients: Dict[str, actionlib.SimpleActionClient] = {}
        self._last_server_name: Optional[str] = None
        if connect_on_init:
            for server_name in server_specs.keys():
                self._connect_to_action_server(server_name)

    def _has_action_result(self, server_name: str) -> bool:
        return self._is_topic_with_suffix(self._namespace, server_name + "/result", "Result")

    def _connect_to_action_server(self, server_name: str, timeout: float = 0.0) -> bool:
        self._last_server_name = server_name
        if server_name not in self._action_clients:
            server_spec = self._server_specs[server_name]
            if not self._has_action_result(server_name):
                if len(server_spec) == 3:
                    self._execute(*server_spec[1:])
                if not self._has_action_result(server_name):
                    rospy.logerr(f"Server '{server_name}' not found by topic.")
                    return False
            action_client = actionlib.SimpleActionClient(self._namespace + server_name, server_spec[0])
            if not action_client.wait_for_server(timeout=Duration(timeout)):
                rospy.logerr(f"Timeout while trying to connect to server '{server_name}'.{' ROS is shutting down.' if rospy.is_shutdown() else ''}")
                return False
            self._action_clients[server_name] = action_client
        return True
    
    def send_goal_and_wait(self, server_name: str, goal: T, timeout: float = 60.0) -> Any:
        return self._action_clients[server_name].send_goal_and_wait(goal, Duration(timeout))

    def send_goal(self, server_name: str, goal: T, done_cb: Any) -> None:
        self._action_clients[server_name].send_goal(
                goal, done_cb
            )
        
    def get_pose_from_goal(self, goal: Any) -> Tuple[List[float], List[float]]:
        return goal.target_pose.pose

    def get_action_result(self, server_name: Optional[str] = None) -> Any:
        if server_name is None:
            server_name = self._last_server_name
            if server_name is None:
                rospy.logerr("Cannot get result before connecting to any server.")
                return None
        if server_name not in self._action_clients:
            rospy.logerr(f"Cannot get result from server '{server_name}'.")
            return None
        return self._action_clients[server_name].get_result()
    
    def lookup_transform(self,
                         reference_frame: str,
                         source_frame: str,
                         time: int = 0) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
        if self._tf_listener is None:
            self._tf_listener = tf.TransformListener()
        return self._tf_listener.lookupTransform(
                reference_frame, source_frame, rospy.Time(time)
            )
    
    def create_move_base_goal(self, pose, frame_id: str) -> MoveBaseGoal:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        return goal

    def create_open_gripper_goal(self, position: float = 0.1, max_effort: float = 100.0) -> GripperCommandGoal:
        goal = GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        return goal
    
    def create_close_gripper_goal(self, position: float = 0.0, max_effort: float = 50.0) -> GripperCommandGoal:
        goal = GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        return goal
    
    def get_move_base_topic_name(self) -> str:
        return self.MOVE_BASE_TOPIC_NAME
    
    def get_moveit_topic_name(self) -> str:
        return self.MOVEIT_TOPIC_NAME
    
    def get_ft_observer_topic_name(self) -> str:
        return self.FT_OBSERVER_TOPIC_NAME
    
    def get_gripper_topic_name(self) -> str:
        return self.GRIPPER_TOPIC_NAME
    
    def wait_for_message(self, topic, message_type, timeout = None):
        return rospy.wait_for_message(topic, message_type, timeout)
    
    def list_params(self, namespace = ""):
        return rosparam.list_params(namespace)
    
    def get_param(self, param_name: str):
        return rosparam.get_param(param_name)
    
    def launch_moveit_macros_command(self) -> str:
        return f"roslaunch robot_api moveit_macros.launch namespace:='{self._namespace.strip('/')}'"


class Ros2Wrapper(RosWrapperInterface):
    MOVE_BASE_TOPIC_NAME = "nav2/navigate_to_pose"
    MOVEIT_TOPIC_NAME = "moveit_macros"
    FT_OBSERVER_TOPIC_NAME = "ft_observer"
    GRIPPER_TOPIC_NAME = "robotiq_gripper_controller/gripper_cmd"
    _ros_node: Optional[Node] = None
    _namespace: str = ""
    _server_specs: Dict[str, Union[Tuple[Generic[T]], Tuple[Generic[T], str, int]]] = {}
    _action_clients: Dict[str, List[ActionClient, Future]] = {}
    _last_server_name: Optional[str] = None
    _tf_listener: Optional[TransformListener] = None
    _tf_buffer: Optional[Buffer] = None

    def _init_node(self) -> Node:
        if self._ros_node is None:
            name = f"robot_api_{os.getpid()}"
            try:
                if not rclpy.ok():
                    rclpy.init()
                self._ros_node = rclpy.create_node(name)
                self.log(f"ROS node '{name}' initialized.", level="debug")
            except rclpy.exceptions.InvalidNodeNameException:
                pass
        elif not rclpy.ok():
            print("ROS is shutting down.")
            return
        return self._ros_node
    
    def get_move_base_topic_name(self) -> str:
        return self.MOVE_BASE_TOPIC_NAME
    
    def get_moveit_topic_name(self) -> str:
        return self.MOVEIT_TOPIC_NAME
    
    def get_ft_observer_topic_name(self) -> str:
        return self.FT_OBSERVER_TOPIC_NAME
    
    def get_gripper_topic_name(self) -> str:
        return self.GRIPPER_TOPIC_NAME

    def log(self, message: str, level: str = "info", *args, **kwargs) -> None:
        node = self._init_node()
        if level == "info":
            node.get_logger().info(message, **kwargs)
        elif level == "warn":
            node.get_logger().warn(message, **kwargs)
        elif level == "error":
            node.get_logger().error(message, **kwargs)
        elif level == "debug":
            node.get_logger().debug(message, **kwargs)
        else:
            node.get_logger().info(message, **kwargs)

    def get_topics(self, namespace: str = ""):
        node = self._init_node()
        return node.get_topic_names_and_types()
    
    def get_action_result(self, server_name: Optional[str] = None) -> Any:
        node = self._init_node()
        if server_name is None:
            server_name = self._last_server_name
            if server_name is None:
                self.log("Cannot get result before connecting to any server.", level="error")
                return None
        if server_name not in self._action_clients:
            self.log(f"Cannot get result from server '{server_name}'.", level="error")
            return None
        return self._action_clients[server_name][1].result()
    
    def init_action_server(self, namespace: str,
                           server_specs: Dict[str, Union[Tuple[Generic[T]], Tuple[Generic[T], str, int]]],
                           connect_on_init: bool = False):
        self._namespace = namespace
        assert server_specs, "Error: You must init an action server with an item in server_specs."
        self._server_specs.update(server_specs)
        self._action_clients: Dict[str, List[ActionClient, Future]] = {}
        self._last_server_name: Optional[str] = None
        if connect_on_init:
            for server_name in server_specs.keys():
                self._connect_to_action_server(server_name)

    def _connect_to_action_server(self, server_name: str, timeout: float = 0.0) -> bool:
        node = self._init_node()
        self._last_server_name = server_name
        if server_name not in self._action_clients:
            server_spec = self._server_specs[server_name]
            action_client = ActionClient(node, server_spec[0], self._namespace + server_name)
            if not action_client.wait_for_server(timeout_sec=0.0) and len(server_spec) == 3:
                self._execute(*server_spec[1:])
            if not action_client.wait_for_server(timeout_sec=timeout):
                self.log(f"Timeout while trying to connect to server '{server_name}'.{' ROS is shutting down.' if not rclpy.ok() else ''}", level="error")
                return False
            self._action_clients[server_name] = [action_client, None]
        return True

    def _has_action_result(self, server_name: str) -> bool:
        return self._is_topic_with_suffix(self._namespace, server_name + "/_action/get_result", "Result")
    
    def lookup_transform(self,
                         reference_frame: str,
                         source_frame: str,
                         time: int = 0) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
        node = self._init_node()
        if self._tf_buffer is None:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, node)
        return self._tf_buffer.lookup_transform(
                reference_frame, source_frame, rclpy.time.Time(seconds=time)
            )
    
    def send_goal_and_wait(self, server_name: str, goal: T, timeout: float = 60.0) -> Any:
        send_goal_future = self._action_clients[server_name][0].send_goal_async(goal)
        rclpy.spin_until_future_complete(self._ros_node, send_goal_future, timeout_sec=timeout/2.0)
        if not send_goal_future.done():
            self.log("Timeout while waiting for goal acceptance.", level="warn")
            return 
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.log("Goal was rejected by server.", level="warn")
            return

        # Wait for result with timeout
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._ros_node, result_future, timeout_sec=timeout)

        if not result_future.done():
            self.log("Result timed out — cancelling goal.", level="warn")
            goal_handle.cancel_goal_async()
            return

        return result_future.result()

    def send_goal(self, server_name: str, goal: T, done_cb: Any) -> None:
        send_goal_future = self._action_clients[server_name][0].send_goal_async(goal)

        send_goal_future.add_done_callback(done_cb)
        self._action_clients[server_name][1] = send_goal_future
        return send_goal_future
    
    def get_pose_from_goal(self, goal: Any) -> Tuple[List[float], List[float]]:
        return goal.pose.pose
    
    def create_move_base_goal(self, pose, frame_id) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self._ros_node.get_clock().now().to_msg()
        goal.pose.pose = pose
        return goal
    
    def create_gripper_goal(self, position: float, max_effort: float = 0.0) -> ParallelGripperCommand.Goal:
        goal = ParallelGripperCommand.Goal()
        goal.command.position = [position, ]
        return goal
    
    def create_open_gripper_goal(self, position: float = 0.0, max_effort: float = 100.0) -> GripperCommandGoal:
        goal = ParallelGripperCommand.Goal()
        goal.command.position = [position, ]
        return goal
    
    def create_close_gripper_goal(self, position: float = 0.755, max_effort: float = 50.0) -> GripperCommandGoal:
        goal = ParallelGripperCommand.Goal()
        goal.command.position = [position, ]
        return goal
    
    def wait_for_message(self, topic, message_type, timeout = None):
        event = Event()
        msg = None

        def callback(msg):
            msg = msg
            event.set()

        sub = self._ros_node.create_subscription(message_type, topic, callback)
        msg_received = event.wait(timeout=timeout)
        self._ros_node.destroy_subscription(sub)

        if not msg_received:
            raise TimeoutError(f"No message received on {topic} within {timeout} seconds.")
        return msg
    
    def list_params(self, namespace = ""):
        return self._ros_node.list_parameters([namespace], 20).names
    
    def get_param(self, param_name: str):
        return self._ros_node.get_parameter(param_name).get_parameter_value()
    
    def launch_moveit_macros_command(self) -> str:
        if self._namespace == "" or self._namespace == "/":
            return "ros2 launch robot_api moveit_macros_ros2.launch.py"
        return f"ros2 launch robot_api moveit_macros_ros2.launch.py namespace:='{self._namespace.strip('/')}'"
    

_ros_wrapper: RosWrapperInterface = None
def get_ros_wrapper() -> RosWrapperInterface:
    global _ros_wrapper
    if _ros_wrapper is None:
        try:
            ros_version = os.environ["ROS_VERSION"]
            if ros_version == "1":
                _ros_wrapper = Ros1Wrapper()
            elif ros_version == "2":
                _ros_wrapper = Ros2Wrapper()
            else:
                raise ImportError(f"Unsupported ROS_VERSION: {ros_version}")
        except KeyError:
            raise ImportError("ROS_VERSION environment variable not set. Please source your ROS setup file.")
    return _ros_wrapper
