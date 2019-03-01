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
#include <geometry_msgs/Twist.h>

#include <radar_sensor_msgs/GPSData.h>
#include <car_data_interface/CarData.h>

#include <radar_slam/geometry_utils.h>

class OdometryBroadcaster {

public:
  OdometryBroadcaster() :
    first_time_( true )
  {
    // Subscribe to the car_speed topic with the updatePose callback:
    sub_car_speed_ = node_handle_.subscribe( "car_data", 10,
                                             &OdometryBroadcaster::updatePose,
                                             this );

    // Load the YAML parameters:
    node_handle_.getParam( "wheelbase", wheelbase );
    node_handle_.getParam( "steering_ratio", steering_ratio );

    // Initialize the pose estimate:
    base_pose_.translation() = Eigen::Vector3d::Zero();
    base_pose_.linear() =
      Eigen::AngleAxisd( 0.0 * M_PI, Eigen::Vector3d::UnitZ() ).toRotationMatrix();
  }
  
  ~OdometryBroadcaster()
  {
  }

  void updatePose( const car_data_interface::CarData &msg )
  {
    // Copy the message in case we want to modify it:
    car_data_interface::CarData car_data_msg = msg;

    // Update the timestep, handling first time carefully:
    ros::Time time_now = ros::Time::now();
    if( first_time_ )
      {
        time_prev_ = time_now;
        first_time_ = false;
      }
    dt_ = ( time_now - time_prev_ ).toSec();

    // Compute average ROUGH wheel speed from message:
    double speed = 0.25 * ( car_data_msg.rough_wheel_speeds.speed_FL +
			    car_data_msg.rough_wheel_speeds.speed_FR +
			    car_data_msg.rough_wheel_speeds.speed_RL +
			    car_data_msg.rough_wheel_speeds.speed_RR );
    
    // Update pose using speed and steering angle:
    processModel( speed, car_data_msg.steer_sensors.steer_angle, dt_, base_pose_, base_vel_, base_yaw_ );

    // Send the updated base pose transform:
    geometry_msgs::TransformStamped pose_tf;
    pose_tf = tf2::eigenToTransform( base_pose_ );
    pose_tf.header.stamp = ros::Time::now();
    pose_tf.child_frame_id = "base_link";
    pose_tf.header.frame_id = "odom";
    bcast_tf_.sendTransform( pose_tf );
    
    time_prev_ = time_now;
  }

  void processModel( double speed, double steer_angle, double dt,
					  Eigen::Affine3d &pose, Eigen::Matrix<double, 6, 1> &vel,
					  double &yaw_unwrpd )
  {
    // Update the orientation using speed and steering angle:
    vel.segment( 3, 3 ) = Eigen::Vector3d(
					  0.0, 0.0,
					  ( speed / wheelbase )
					  * tan( ( M_PI / 180.0 ) * ( steer_angle / steering_ratio ) ) );
    pose.linear() = Eigen::AngleAxisd( dt * vel( 5 ), Eigen::Vector3d::UnitZ() ).toRotationMatrix()
      * pose.linear();

    // Update position using speed and updated orientation:
    yaw_unwrpd = geometry_utils::unwrap( yaw_unwrpd, pose.linear().eulerAngles( 0, 1, 2 )( 2 ) );
    vel.segment( 0, 3 ) = Eigen::Vector3d( speed * cos( yaw_unwrpd ), speed * sin( yaw_unwrpd ),
                                           0.0 );
    pose.translation() += dt * vel.segment( 0, 3 );
  }
    
  // Parameters loaded from YAML file radar_slam/config/radar_slam.yaml:
  /**
   * Wheelbase of the testbed, in meters
   *
   * This was obtained for the Acura ILX 2016 from digging into the OpenPilot codebase.
   */
  double wheelbase;
  /**
   * Steering ratio of the testbed, dimensionless
   *
   * This relates the steering angle to the wheel angle (effectively a steering sensitivity).
   */
  double steering_ratio;
  
private:
  /**
   * Previous timestamp measured by ROS, used to compute timestep (dt).
   */
  ros::Time time_prev_;
  /**
   * Timestep used for updates.
   */
  double dt_;
  /**
   * Set to true on construction of the class, set to false after first update.
   */
  bool first_time_;

  ros::NodeHandle node_handle_;
  ros::Subscriber sub_car_speed_;

  // State variables:
  /**
   * Pose (6d position+orientation) of the base (root link) of the car.
   */
  Eigen::Affine3d base_pose_;
  /**
   * Unwrapped yaw angle of the car, in radians.
   */
    double base_yaw_;
  /**
   * Velocity (6d linear+angular) vector of the base of the car.
   */
  Eigen::Matrix<double, 6, 1> base_vel_;

  tf2_ros::TransformBroadcaster bcast_tf_;
};

int main( int argc, char** argv )
{
  ros::init( argc, argv, "base_link_tf_broadcaster" );

  OdometryBroadcaster odom_bcast;

  ros::spin();
}
