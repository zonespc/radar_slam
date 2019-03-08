## radar_slam:
Implementation of SLAM using automotive radars. Currently under development.

## Running the car demo

### Network setup

For now, use radar with IP 10.0.0.13 on FRONT LEFT corner, and use radar with IP 10.0.0.10
on FRONT RIGHT corner of car.  Both should face outwards (parallel to car body). The TX2 static IP must
be set to 10.0.0.75, this can be done by replacing the existing /etc/network/interfaces file with:

```bash
auto lo
iface lo inet loopback

auto eth0
iface eth0 inet static
address 10.0.0.75
netmask 255.255.255.0
```

Once the radars are powered and connected, try ```ping 10.0.0.10``` and ```ping 10.0.0.13```.  If you cannot
ping either of the radars then something is wrong with the network setup.

### Installing software

You will need this package in addition to [radar_ros_interface](https://github.com/AinsteinAI/radar_ros_interface),
[radar_sensor_msgs](https://github.com/AinsteinAI/radar_sensor_msgs) and [rviz_radar_plugin](https://github.com/AinsteinAI/rviz_radar_plugin).

Create a catkin workspace with

```bash
cd ~/
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/AinsteinAI/radar_ros_interface
git clone https://github.com/AinsteinAI/radar_sensor_msgs
git clone https://github.com/AinsteinAI/radar_slam
git clone https://github.com/AinsteinAI/rviz_radar_plugin
git clone https://github.com/ethz-asl/ethz_piksi_ros
cd ..
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
catkin build
```

If the build fails then you may be missing some dependencies:

#### PCL-related packages

Install using apt: ```ros-kinetic-pcl-conversions ros-kinetic-pcl-msgs ros-kinetic-pcl-ros```

#### tf2_eigen

Required for PCL, install using apt: ```sudo apt install ros-kinetic-tf2-eigen```
#### Eigen library

This is installed as below:

```bash
cd /tmp # or anywhere else you want to build from
wget http://bitbucket.org/eigen/eigen/get/3.3.7.tar.bz2 # latest version as of writing is 3.3.7
tar xvjf 3.3.7.tar.bz2
cd eigen-eigen-323c052e1731
mkdir build_dir && cd build_dir
cmake .. && sudo make install
```
#### Piksi SBP Communication Library

Install this using the script in the ethz_piksi_ros package:

```bash
cd ~/catkin_ws/src/ethz_piksi_ros/piksi_multi_rtk_ros/install/
source install_piksi_multi.sh
```

After installing SBP and building the catkin packages, you can try running the Piksi on its own:

```bash
roslaunch piksi_v2_rtk_ros piksi_rover.launch
```

If this succeeds you should be able to run ```rostopic list``` and see a bunch of piksi-related topics. Note
that the blue light on the Piksi board must be solid to know that the GPS fix is acquired. Make sure the antenna
is in a good spot.

### Running the demo

In ROS, a *launch file* runs a bunch of different *nodes* (processes) at once.  The launch file for the demo is
```gps_pose_est.launch```.  It runs ```gps_pose_est_base.launch``` with some parameters set which determine what
components (camera, GPS, radars etc) to use. The ```gps_pose_est_base.launch``` file included other launch files
to run these components, and there are parameters in each of those which affect performance.

All you need to do is run the following command:

```bash
roslaunch radar_slam gps_pose_est.launch
```

to run the whole demo.  This should run the Piksi GPS node, the radar nodes and Rviz. Once Rviz opens, the panel on
the left lets you customize the visualization.