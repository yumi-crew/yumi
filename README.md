# ROS 2 stack for external control of the ABB YuMi.



Installation:
~~~~
mkdir -p /home/$USER/abb_ws/src
# Clone repo into src
vcs import /home/$USER/abb_ws/src < /home/$USER/abb_ws/src/yumi/yumi.repos
cd /home/$USER/abb_ws/
colcon build --symlink-install
source install/local_setup.bash`
~~~~
