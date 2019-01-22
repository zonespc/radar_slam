## radar_slam:
Implementation of SLAM using automotive radars. Currently under development.

### ROS	Dependencies:

As always, running:

```bash
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```

Should install all ROS dependencies.  These include:

PCL-related packages - Install using apt: ```ros-kinetic-pcl-conversions ros-kinetic-pcl-msgs ros-kinetic-pcl-ros```
tf2_eigen - Required for PCL, install using apt: ```sudo apt install ros-kinetic-tf2-eigen```

