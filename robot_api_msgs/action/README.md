Note
====

The action definitions of `robot_api` must be copies of what other
actionlib servers provide in the ROS system and `robot_api` is connecting to.
Copies are meant with regard to their binary contents, so their checksums match.

ROS 1 vs. ROS 2
---------------

ROS 2 (`rosidl`) only accepts snake_case field names, so a definition
cannot always be byte-identical for both ROS versions. In that case:

- `action/<Name>.action` is the ROS 2 definition.
- `action/ros1/<Name>.action` is the ROS 1 definition, a byte-identical copy
  of the server's definition
  (e.g. `mobipick_pick_n_place/action/FtObserver.action`).

Definitions that are identical for both versions live only in `action/`.
