# ROS 2 stack for external control of the ABB YuMi.
<p align="center">
  <img src="https://github.com/yumi-crew/yumi/blob/eloquent/yumi_description/meshes/yumi_render.png" width=300>
</p>


### Dependencies:

* abb_librws dependencies
   * POCO C++ Libraries (>= 1.4.3 due to WebSocket support)
       * Navigate to home dir.
         Should be built from source.
         ~~~~
         sudo apt-get -y update && sudo apt-get -y install git g++ make cmake libssl-dev  
         git clone -b master https://github.com/pocoproject/poco.git   
         cd poco
         mkdir cmake-build 
         cd cmake-build
         sudo cmake ..
         sudo cmake --build . --target install
         ~~~~
* abb_libegm dependencies
  * Boost
     * `sudo apt-get install libboost-all-dev`
  * Protobuf 
     * Should be built from source.
        ~~~~ 
        sudo apt-get install autoconf automake libtool curl make g++ unzip  
        git clone https://github.com/protocolbuffers/protobuf.git  
        cd protobuf  
        ./configure
        sudo make
        sudo make install
        sudo ldconfig # refresh shared library cache.
        ~~~~ 
* Install pose_estimation dependencies. Guide in repo https://github.com/yumi-crew/pose_estimation
* Install zivid-ros dependencies. Guide in repoo https://github.com/yumi-crew/zivid-ros.git

     
### Installation:

Installs ROS 2 packages used to control the ABB YuMi and the Zivid One 3D camera. External packages are contained in the organization. 


1. Clone repos
    ~~~~
    mkdir -p /home/$USER/abb_ws/src && cd /home/$USER/abb_ws/src
    git clone https://github.com/yumi-crew/yumi.git
    vcs import /home/$USER/abb_ws/src < /home/$USER/abb_ws/src/yumi/yumi.repos
    vcs import /home/$USER/abb_ws/src < /home/$USER/abb_ws/src/moveit2/moveit2.repos
    rosdep install -r --from-paths . --ignore-src --rosdistro eloquent -y
    ~~~~
    
2. Build

    ~~~~
    # To build moveit2
    colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
    # Normal build, just in case;)
    colcon build --symlink-install
    source install/local_setup.bash
    ~~~~

### Use 

Launch architecture
   ~~~~ 
   ros2 launch yumi_launch yumi_moveit2_zivid.launch.py
   ~~~~
   Sim:
   ~~~~
   ros2 launch yumi_launch yumi_moveit2_zivid_sim.launch.py
   ~~~~
Run bin pick demo
   ~~~~
   ros2 launch yumi_test demo.launch.py
   ~~~~
