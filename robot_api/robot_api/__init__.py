try:
    import os

    ros_version = os.environ["ROS_VERSION"]
    if ros_version == "1":
        import robot_api_msgs.msg
    elif ros_version == "2":
        import robot_api_msgs.action
    else:
        raise ImportError(f"Unsupported ROS_VERSION: {ros_version}")
except KeyError:
    raise ImportError("ROS_VERSION environment variable not set. Please source your ROS setup file.")
except ModuleNotFoundError:
    print(
        "Error: Trying to import robot_api without its ROS environment does not work."
    )
    print("Note: Calling 'import robot_api' at repository directory level")
    print(" will cause such a local import of the 'robot_api' directory.")
    print(" Do import robot_api from somewhere else.")
    raise


from robot_api.lib import (
    TuplePose,
    is_instance,
    get_at,
    get_angle_between,
    get_pose_name,
    find_robot_namespaces,
    add_waypoint,
    save_waypoints,
    load_waypoints,
    print_waypoints,
)
from robot_api.core import Robot, Base
from robot_api.extensions import Arm
from robot_api.excepthook import Excepthook
