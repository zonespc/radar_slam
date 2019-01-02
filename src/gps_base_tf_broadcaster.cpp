/*
  Copyright <2018> <Ainstein, Inc.>

  Redistribution and use in source and binary forms, with or without modification, are permitted 
  provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this list of 
  conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this list of 
  conditions and the following disclaimer in the documentation and/or other materials provided 
  with the distribution.

  3. Neither the name of the copyright holder nor the names of its contributors may be used to 
  endorse or promote products derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR 
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND 
  FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
  OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <eigen3/Eigen/Eigen>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>

#include <radar_sensor_msgs/GPSData.h>

class GPSPoseEstimator {

public:
  GPSPoseEstimator()
  {
    // Register the callback:
    sub_gps_data_ = node_handle_.subscribe( "/gps_data", 10,
					    &GPSPoseEstimator::updatePose,
					    this );

    // Initialize the pose estimate:
    base_pose_.translation() = Eigen::Vector3d::Zero();
    base_pose_.linear() =
      Eigen::AngleAxisd( 0.0 * M_PI, Eigen::Vector3d::UnitZ() ).toRotationMatrix();

    // Create the mapping from NED frame to world frame in ROS:
    ned_to_world_.translation() = Eigen::Vector3d::Zero();
    ned_to_world_.linear() =
      Eigen::AngleAxisd( M_PI, Eigen::Vector3d::UnitY() ).toRotationMatrix();
    
    first_time_ = true;
  }
  ~GPSPoseEstimator(){}
  
  void updatePose( const radar_sensor_msgs::GPSData &msg )
  {
    // Compute the dt:
    if( first_time_ )
      {
	gps_time_prev_ = msg.gps_time;
	first_time_ = false;
      }
    double dt = msg.gps_time - gps_time_prev_;
    Eigen::Vector3d vel_ned = Eigen::Vector3d( msg.velocity_ned.x,
					       msg.velocity_ned.y,
					       0.0 );

    // Transform the GPS velocity from NED to world frame:
    Eigen::Vector3d vel_world = ned_to_world_ * vel_ned;
    
    // Integrate the estimated pose:
    base_pose_.translation() += dt * vel_world;

    // Compute the yaw angle from velocity direction when moving:
    if( vel_world.norm() >= 0.1 )
      {
	base_pose_.linear() =
	  Eigen::AngleAxisd( atan2( vel_world(1), vel_world(0) ), Eigen::Vector3d::UnitZ() ).toRotationMatrix();
     }
    
    // Send the updated base pose transform:
    geometry_msgs::TransformStamped pose_tf;
    ros::Time time_now = ros::Time::now();
    pose_tf = tf2::eigenToTransform( base_pose_ );
    pose_tf.header.stamp = time_now;
    pose_tf.child_frame_id = "chassis";
    pose_tf.header.frame_id = "base_link";
    bcast_tf_.sendTransform( pose_tf );

    // Store the GPS time:
    gps_time_prev_ = msg.gps_time;
  }
  
private:
  ros::NodeHandle node_handle_;
  ros::Subscriber sub_gps_data_;
  
  Eigen::Affine3d ned_to_world_;
  Eigen::Affine3d base_pose_;
  tf2_ros::TransformBroadcaster bcast_tf_;
  
  double gps_time_prev_;
  bool first_time_;

};

int main( int argc, char** argv )
{
  ros::init( argc, argv, "base_link_tf_broadcaster" );

  GPSPoseEstimator gps_est;

  ros::spin();
}
