# Dependency
`sudo apt install -y ros-melodic-desktop-full python-rosdep python-rosinstall ros-melodic-apriltag-ros ros-melodic-lms1xx ros-melodic-jackal-navigation ros-melodic-robot-localization`

# Build
`catkin_make`
`source devel/setup.bash`

# Launch
Start with `roslaunch start.launch` and `python src/preprocessing/preprocessing.py` (python2 version).

# Getting Start
After `Launch` part, you can control your robot by `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` (ensure that you install the corresponding dependency first!).



