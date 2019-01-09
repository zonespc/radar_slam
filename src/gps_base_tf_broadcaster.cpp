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

#include <thread>
#include <mutex>
#include <eigen3/Eigen/Eigen>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/Twist.h>

#include <radar_sensor_msgs/GPSData.h>

class GPSPoseEstimator {

public:
  GPSPoseEstimator()
  {
    // Register the callback:
    sub_gps_data_ = node_handle_.subscribe( "/gps_data", 10,
					    &GPSPoseEstimator::updateGPSData,
					    this );

    // Get the heading LPF coefficient:
    node_handle_.param( "heading_lpf", alpha_, 0.1 );
    
    // Initialize the pose estimate:
    base_pose_.translation() = Eigen::Vector3d::Zero();
    base_pose_.linear() =
      Eigen::AngleAxisd( 0.0 * M_PI, Eigen::Vector3d::UnitZ() ).toRotationMatrix();

    // Create the mapping from NED frame to world frame in ROS:
    ned_to_world_.translation() = Eigen::Vector3d::Zero();
    ned_to_world_.linear() =
      Eigen::AngleAxisd( M_PI, Eigen::Vector3d::UnitY() ).toRotationMatrix();

    // Create and launch the pose update thread:
    thread_ = std::unique_ptr<std::thread>( new std::thread( &GPSPoseEstimator::updatePose, this, 1000.0 ) );
    mutex_.lock();
    is_running_ = true;
    mutex_.unlock();

    // Initialize the car velocity publisher:
    pub_car_vel_ = node_handle_.advertise<geometry_msgs::Twist>( "car_vel", 10 );
    
    // Set first time true for computing delta time:
    first_time_ = true;
  }
  
  ~GPSPoseEstimator()
  {
    mutex_.lock();
    is_running_ = false;
    mutex_.unlock();

    thread_->join();
  }
  
  void updateGPSData( const radar_sensor_msgs::GPSData &msg )
  {
    mutex_.lock();
    gps_msg_ = msg;
    mutex_.unlock();
  }

  void updatePose( double freq )
  {
    // Local variables:
    radar_sensor_msgs::GPSData msg;
    Eigen::Vector3d vel_ned_prev = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_ned;

    // Create ROS rate for running at desired frequency:
    ros::Rate update_pose_rate( freq );
    
    // Enter the main loop:
    bool running = true;
    while( running && ros::ok() && !ros::isShuttingDown() )
      {
	// Compute the dt:
	if( first_time_ )
	  {
	    time_prev_ = ros::Time::now();
	    first_time_ = false;
	  }

	// Get the latest GPS data:
	mutex_.lock();
	msg = gps_msg_;
	mutex_.unlock();

	// Compute the actual delta time:
	ros::Time time_now = ros::Time::now();
	double dt = ( time_now - time_prev_ ).toSec();

	// Get the latest GPS data and low pass filter it:
	vel_ned = alpha_ * Eigen::Vector3d( msg.velocity_ned.x,
					   msg.velocity_ned.y,
					   0.0 ) + ( 1.0 - alpha_ ) * vel_ned;
	
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
	pose_tf = tf2::eigenToTransform( base_pose_ );
	pose_tf.header.stamp = ros::Time::now();
	pose_tf.child_frame_id = "chassis";
	pose_tf.header.frame_id = "base_link";
	bcast_tf_.sendTransform( pose_tf );

	// Publish the car velocity:
	geometry_msgs::Twist car_vel_msg;
	tf2::toMsg( vel_world, car_vel_msg.linear );
	tf2::toMsg( Eigen::Vector3d::Zero(), car_vel_msg.angular );
	pub_car_vel_.publish( car_vel_msg ); 
	
	// Store the current time and velocity:
	time_prev_ = time_now;
	vel_ned_prev = Eigen::Vector3d( msg.velocity_ned.x,
					msg.velocity_ned.y,
					0.0 );
	
	// Check whether the data loop should still be running:
	mutex_.lock();
	running = is_running_;
	mutex_.unlock();

	// Sleep to maintain desired freq:
	update_pose_rate.sleep();	
      }
  }
    
  
private:
  ros::NodeHandle node_handle_;
  ros::Subscriber sub_gps_data_;
  radar_sensor_msgs::GPSData gps_msg_;

  double alpha_;
  
  ros::Publisher pub_car_vel_;
  
  Eigen::Affine3d ned_to_world_;
  Eigen::Affine3d base_pose_;
  tf2_ros::TransformBroadcaster bcast_tf_;

  bool is_running_;
  std::unique_ptr<std::thread> thread_;
  std::mutex mutex_;
  
  ros::Time time_prev_;
  bool first_time_;

};

int main( int argc, char** argv )
{
  ros::init( argc, argv, "base_link_tf_broadcaster" );

  GPSPoseEstimator gps_est;

  ros::spin();
}
