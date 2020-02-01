# ROS 2 integration of ABB IRB14000 YuMi**



Installation:

  `mkdir -p /home/$USER/abb_ws/src
  # Clone repo into src
  vcs import /home/$USER/abb_ws/src < /home/$USER/abb_ws/src/abb_egm_hardware/abb_egm_hardware.repos
  cd /home/$USER/abb_ws/
  colcon build --symlink-install
  source install/local_setup.bash` 