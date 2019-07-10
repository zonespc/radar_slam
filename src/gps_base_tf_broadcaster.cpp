/*
  Copyright <2018-2019> <Ainstein, Inc.>

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
#include <geometry_msgs/TwistStamped.h>
#include <piksi_rtk_msgs/VelNed.h>
#include <radar_sensor_msgs/GPSData.h>

class GPSPoseEstimator {

public:
  GPSPoseEstimator() :
    nh_private_( "~" )
  {
    // Register the callback for radar-routed GPS data (DEPRECATED, TO BE REMOVED):
    sub_gps_data_radar_ = nh_.subscribe( "gps_data", 10,
					 &GPSPoseEstimator::updateGPSDataRadar,
					 this );

    // Register the callback for RAC GPS data:
    sub_gps_data_rac_ = nh_.subscribe( "/vel", 10,
				       &GPSPoseEstimator::updateGPSDataRAC,
				       this );
    
    // Register the callback for piksi GPS data:
    sub_gps_data_piksi_ = nh_.subscribe( "/piksi/vel_ned", 10,
					 &GPSPoseEstimator::updateGPSDataPiksi,
					 this );
    
    // Get parameters:
    nh_private_.param( "gps_vel_lpf", gps_vel_lpf_, 0.1 );
    nh_private_.param( "integration_rate", integration_rate_, 1000.0 );
    nh_private_.param( "publish_tf", publish_tf_, true );
    nh_private_.param( "publish_pose_stamped", publish_pose_stamped_, true );

    // Initialize the GPS NED velocity, to be updated from GPS node:
    gps_vel_ned_.setZero();
    
    // Initialize the pose estimate:
    base_pose_.translation() = Eigen::Vector3d::Zero();
    base_pose_.linear() =
      Eigen::AngleAxisd( 0.0 * M_PI, Eigen::Vector3d::UnitZ() ).toRotationMatrix();

    // Create the mapping from NED frame to world frame in ROS:
    ned_to_world_.translation() = Eigen::Vector3d::Zero();
    ned_to_world_.linear() =
      Eigen::AngleAxisd( M_PI, Eigen::Vector3d::UnitY() ).toRotationMatrix();
    
    // Create and launch the pose update thread:
    thread_ = std::unique_ptr<std::thread>( new std::thread( &GPSPoseEstimator::updatePose, this, integration_rate_ ) );
    mutex_.lock();
    is_running_ = true;
    mutex_.unlock();
    
  }
  
  ~GPSPoseEstimator()
  {
    mutex_.lock();
    is_running_ = false;
    mutex_.unlock();

    thread_->join();
  }
  
  void updateGPSDataRadar( const radar_sensor_msgs::GPSData &msg )
  {
    mutex_.lock();

    gps_vel_ned_ = Eigen::Vector3d( msg.velocity_ned.x,
				    msg.velocity_ned.y,
				    msg.velocity_ned.z );

    mutex_.unlock();
  }
  
  void updateGPSDataPiksi( const piksi_rtk_msgs::VelNed &msg )
  {
    mutex_.lock();

    // Copy and scale Piksi GPS data (NED frame):
    gps_vel_ned_ = 0.001 * Eigen::Vector3d( static_cast<double>( msg.n ),
					    static_cast<double>( msg.e ),
					    static_cast<double>( msg.d ) );

    mutex_.unlock();
  }

  void updateGPSDataRAC( const geometry_msgs::TwistStamped &msg )
  {
    mutex_.lock();
    
    // Convert the RAC message from ENU to NED and check for NaN:
    gps_vel_ned_.x() = std::isnan( msg.twist.linear.y ) ? 0.0 : msg.twist.linear.y;
    gps_vel_ned_.y() = std::isnan( msg.twist.linear.x ) ? 0.0 : msg.twist.linear.x;
    gps_vel_ned_.z() = 0.0;

    mutex_.unlock();
  }
  
  void updatePose( double freq )
  {
    // Local variables:
    Eigen::Vector3d vel_ned = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_ned_lpf = vel_ned; // low pass filtered
    bool first_time = true;  
    ros::Time time_now, time_prev;
    double dt;

    // Initialize the car velocity publisher:
    pub_car_vel_ = nh_private_.advertise<geometry_msgs::Twist>( "car_vel", 1000 );

    // Initialize the pose publisher:
    pub_pose_stamped_ = nh_private_.advertise<geometry_msgs::PoseStamped>( "base_link_pose", 1000 );
    
    // Create ROS rate for running at desired frequency:
    ros::Rate update_pose_rate( freq );
    
    // Enter the main loop:
    bool running = true;
    while( running && ros::ok() && !ros::isShuttingDown() )
      {
	// Compute the dt:
	if( first_time )
	  {
	    time_prev = ros::Time::now();

	    // Wait for simulated clock to start:
	    if( time_prev.toSec() > 0.0 )
	      {
		first_time = false;
	      }
	  }

	// Get the latest GPS data:
	mutex_.lock();
	vel_ned = gps_vel_ned_;
	mutex_.unlock();

	// Ignore the z direction velocity for now, keep everything 2d:
	vel_ned(2) = 0.0;

	// Compute the actual delta time:
	time_now = ros::Time::now();
	dt = ( time_now - time_prev ).toSec();

	// Get the latest GPS velocity and low pass filter it, ignoring z for now:
	vel_ned_lpf = gps_vel_lpf_ * vel_ned + ( 1.0 - gps_vel_lpf_ ) * vel_ned_lpf;
	
	// Transform the GPS velocity from NED to world frame:
	Eigen::Vector3d vel_world = ned_to_world_ * vel_ned_lpf;

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
	pose_tf.child_frame_id = "base_link";
	pose_tf.header.frame_id = "map";
	if( publish_tf_ )
	  {
	    bcast_tf_.sendTransform( pose_tf );
	  }

	// Send the updated base pose message:
	geometry_msgs::PoseStamped pose;
	pose.pose = tf2::toMsg( base_pose_ );
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "map";
	if( publish_pose_stamped_ )
	  {
	    pub_pose_stamped_.publish( pose );
	  }

	// Publish the car velocity:
	geometry_msgs::Twist car_vel_msg;
	tf2::toMsg( vel_world, car_vel_msg.linear );
	tf2::toMsg( Eigen::Vector3d::Zero(), car_vel_msg.angular );
	pub_car_vel_.publish( car_vel_msg ); 
	
	// Store the current time and velocity:
	time_prev = time_now;
	
	// Check whether the data loop should still be running:
	mutex_.lock();
	running = is_running_;
	mutex_.unlock();

	// Sleep to maintain desired freq:
	update_pose_rate.sleep();	
      }
  }
    
  
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  
  ros::Subscriber sub_gps_data_radar_;
  ros::Subscriber sub_gps_data_piksi_;
  ros::Subscriber sub_gps_data_rac_;
  Eigen::Vector3d gps_vel_ned_;
  
  double gps_vel_lpf_;
  double integration_rate_;
  bool publish_tf_;
  bool publish_pose_stamped_;
  
  ros::Publisher pub_car_vel_;
  ros::Publisher pub_pose_stamped_;
  
  Eigen::Affine3d ned_to_world_;
  Eigen::Affine3d base_pose_;
  tf2_ros::TransformBroadcaster bcast_tf_;

  bool is_running_;
  std::unique_ptr<std::thread> thread_;
  std::mutex mutex_;

};

int main( int argc, char** argv )
{
  ros::init( argc, argv, "base_link_tf_broadcaster" );

  GPSPoseEstimator gps_est;

  ros::spin();
}
