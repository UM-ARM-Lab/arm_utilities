# arm_utilities
C++ and Python utilities used in lab projects. Renamed from the old name, `arc_utilities`.

# Rough list of utilities

- Converting between `geometry_msgs.Transform` and `np.array` types
  - `numpy_conversions.py`
- A fast specialized matrix inversion for 4x4 homogeneous transforms, and spherical coordinate conversions
  - `transformation_helper.py`
- Stopping long blocking functions on a timeout
  - `catch_timeout.py`
- Getting the latest message on a topic, assuming a multi-threaded rclpy setup.
  - `listener.py`

# ROS 2 Upgrade

This repo has not been used much recently, so during the ROS 2 upgrade most of the code was deleted instead of upgraded. See the old `noetic` branch for the more complete ROS 1 version.