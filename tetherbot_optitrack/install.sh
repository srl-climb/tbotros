# Change Directory into your workspace e.g. ros_ws/src
# If vcs is not installed:
# sudo apt install python3-vcstool

git clone https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack.git
git clone https://github.com/MOCAP4ROS2-Project/mocap.git
git clone https://github.com/MOCAP4ROS2-Project/mocap_msgs.git

vcs import < mocap/dependency_repos.repos
cd .. && rosdep install --from-paths src --ignore-src -r -y

# Building with symlinks is necessary for mocap, dont forget to source or open a new terminal
colcon build --symlink-install