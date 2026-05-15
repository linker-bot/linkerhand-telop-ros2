colcon build --symlink-install
source install/setup.bash

启动分为两种，
一种带标定
ros2 run linkerhand_retarget handretarget --ros-args -p calibration := True
一种正常启动，默认不执行标定
ros2 run linkerhand_retarget handretarget 
还是这些流程