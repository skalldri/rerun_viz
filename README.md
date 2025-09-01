# rerun_viz
A replacement for rviz2, built using [Rerun](https://rerun.io/).

## Why replace rviz?
To put it delicately, rviz hasn't been significantly updated in the 15 years I've been using it. The UI is dated, the camera controls are hard to use. Medium-sized point clouds can cause the UI to grind to a halt. rviz can easily crash if given invalid data.

I use Rerun professionally, and think it's a significantly better tool than rviz. I want to use Rerun with my own robots instead of rviz, so I'm making it happen.

## Why not [`rerun-ros`](https://github.com/rerun-io/rerun-ros)?
In a nutshell: it's written in Rust, and the barrier to entry is too high to use Rust with ROS2.

I tried to get the `rerun-ros` repo to work, but hit multiple roadblocks:

- `rclrs` is required. As far as I can tell, this currently needs to be compiled from source. I could not find precompiled packages for `rclrs` (as of ROS2 Jazzy). This is a tall ask.
- `colcon-cargo` and `colcon-ros-cargo` extensions are required to build Rust code with ROS2, but these Colcon extensions are not published to the Ubuntu/ROS2 package repositories. Ubuntu 24.04 (the officially supported Linux distro for ROS2 Jazzy) does not support installing Python packages via `pip` anymore. This requires creating a Python `venv` for compiling anything ROS2-Rust, which is a significant burden and departure from the official ROS2 documentation
- Unclear message-binding support for `rclrs`: as I experienced firsthand in my previous work to modernize `rcljava`, getting message binding generation to work for anything other than `rclcpp` is very difficult, and typically requries a source-checkout. I don't want to spend time debugging message generation, I want to spend time writing visualizer code.

It's a noble goal to write code in memory-safe languages like Rust, but for projects that need to integrate into an ecosystem, it's better to use the common language of the ecosystem. For ROS2, those languages are either Python or C++. I'll use C++ for performance (which is one reason `rerun-ros` picked Rust over Python).

Since the goal of this project is to supplant `rviz` as the standard visualization tool for ROS2, ease of use must be a primary goal.