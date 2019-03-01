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
#include <sensor_msgs/JointState.h>

#include <car_data_interface/CarData.h>

class CarJointStatePub {

public:
  CarJointStatePub( void ) :
    nh_private_( "~" )
  {
    // Register the callback:
    sub_car_data_ = nh_.subscribe( "car_data", 10,
				   &CarJointStatePub::updateCarJointState,
				   this );
    
    // Get parameters (defaults for Acura ILX 2016):
    nh_private_.param( "steering_ratio", steering_ratio_, 18.61 );
    nh_private_.param( "wheel_radius", wheel_radius_, 0.31242 );

    // Integration rate for wheel speeds:
    nh_private_.param( "integration_rate", integration_rate_, 1000.0 );
    
    // Create and launch the pose update thread:
    thread_ = std::unique_ptr<std::thread>( new std::thread( &CarJointStatePub::publishLoop, this, integration_rate_ ) );
    mutex_.lock();
    is_running_ = true;
    mutex_.unlock();

    // Initialize the joint state and publisher:
    joint_state_msg_.name.resize( 7 );
    joint_state_msg_.name.at( STEER_WHEEL ) = std::string( "steering_joint" );
    joint_state_msg_.name.at( FL_STEER ) = std::string( "front_left_steer_joint" );
    joint_state_msg_.name.at( FR_STEER ) = std::string( "front_right_steer_joint" );
    joint_state_msg_.name.at( FL_WHEEL ) = std::string( "front_left_wheel_joint" );
    joint_state_msg_.name.at( FR_WHEEL ) = std::string( "front_right_wheel_joint" );
    joint_state_msg_.name.at( RL_WHEEL ) = std::string( "rear_left_wheel_joint" );
    joint_state_msg_.name.at( RR_WHEEL ) = std::string( "rear_right_wheel_joint" );

    joint_state_msg_.position.resize( 7 );
    std::fill( joint_state_msg_.position.begin(), joint_state_msg_.position.end(), 0.0 );

    joint_state_msg_.velocity.resize( 7 );
    std::fill( joint_state_msg_.velocity.begin(), joint_state_msg_.velocity.end(), 0.0 );
  }
  
  ~CarJointStatePub()
  {
    mutex_.lock();
    is_running_ = false;
    mutex_.unlock();

    thread_->join();
  }
  
  void updateCarJointState( const car_data_interface::CarData &msg )
  {
    mutex_.lock();
    car_data_msg_ = msg;
    mutex_.unlock();
  }

  void publishLoop( double freq )
  {
    // Local variables:
    car_data_interface::CarData msg;
    ros::Time time_prev;
    bool first_time = true;

    // Initialize the joint state publisher:
    pub_joint_state_ = nh_.advertise<sensor_msgs::JointState>( "joint_states", 1000 );
    
    // Create ROS rate for running at desired frequency:
    ros::Rate publish_loop_rate( freq );
    
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

	// Get the latest car data:
	mutex_.lock();
	msg = car_data_msg_;
	mutex_.unlock();

	// Compute the actual delta time:
	ros::Time time_now = ros::Time::now();
	double dt = ( time_now - time_prev ).toSec();

	// Set steering wheel angle (convert to radians):
	joint_state_msg_.position.at( STEER_WHEEL ) = -( M_PI / 180.0 ) * msg.steer_sensors.steer_angle;

	// Set the steering angle for the front wheels:
	joint_state_msg_.position.at( FL_STEER ) = -( M_PI / 180.0 ) * ( msg.steer_sensors.steer_angle / steering_ratio_ );
	joint_state_msg_.position.at( FR_STEER ) = -( M_PI / 180.0 ) * ( msg.steer_sensors.steer_angle / steering_ratio_ );

	// Compute the wheel instantaneous rotation rate (velocity):
	joint_state_msg_.velocity.at( FL_WHEEL ) = msg.rough_wheel_speeds.speed_FL / wheel_radius_;
	joint_state_msg_.velocity.at( FR_WHEEL ) = msg.rough_wheel_speeds.speed_FR / wheel_radius_;
	joint_state_msg_.velocity.at( RL_WHEEL ) = msg.rough_wheel_speeds.speed_RL / wheel_radius_;
	joint_state_msg_.velocity.at( RR_WHEEL ) = msg.rough_wheel_speeds.speed_RR / wheel_radius_;
	
	// Integrate continuous wheel rotation from rotation rate:
	joint_state_msg_.position.at( FL_WHEEL ) += dt * joint_state_msg_.velocity.at( FL_WHEEL );
	joint_state_msg_.position.at( FR_WHEEL ) += dt * joint_state_msg_.velocity.at( FR_WHEEL );
	joint_state_msg_.position.at( RL_WHEEL ) += dt * joint_state_msg_.velocity.at( RL_WHEEL );
	joint_state_msg_.position.at( RR_WHEEL ) += dt * joint_state_msg_.velocity.at( RR_WHEEL );
	
	// Publish the updated joint state message:
	joint_state_msg_.header.stamp = time_now;
	pub_joint_state_.publish( joint_state_msg_ ); 
	
	// Store the current time:
	time_prev = time_now;
	
	// Check whether the data loop should still be running:
	mutex_.lock();
	running = is_running_;
	mutex_.unlock();

	// Sleep to maintain desired freq:
	publish_loop_rate.sleep();
      }
  }

    // Constants for indexing into JointState message:
    static const int STEER_WHEEL = 0;
    static const int FL_STEER = 1;
    static const int FR_STEER = 2;
    static const int FL_WHEEL = 3;
    static const int FR_WHEEL = 4;
    static const int RL_WHEEL = 5;
    static const int RR_WHEEL = 6;
    
private:
  double steering_ratio_;
  double wheel_radius_;
  double integration_rate_;
  
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber sub_car_data_;
  car_data_interface::CarData car_data_msg_;
  ros::Publisher pub_joint_state_;
  sensor_msgs::JointState joint_state_msg_;

  bool is_running_;
  std::unique_ptr<std::thread> thread_;
  std::mutex mutex_;

};

int main( int argc, char** argv )
{
  ros::init( argc, argv, "car_joint_state_pub_node" );

  CarJointStatePub joint_state_pub;

  ros::spin();
}
