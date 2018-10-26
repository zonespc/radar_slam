#ifndef CAR_POSE_ESTIMATOR_H_
#define CAR_POSE_ESTIMATOR_H_

#include <cmath>
#include <random>
#include <iostream>
#include <thread>

#include <eigen3/Eigen/Eigen>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Range.h>
#include <tf2/LinearMath/Quaternion.h>

#include <radar_slam/CarData.h>
#include <radar_ros_interface/RadarData.h>

class CarPoseEstimator
{
public:
    CarPoseEstimator();
    ~CarPoseEstimator()
    {
    }

    void processModel( double speed, double steer_angle, double dt, Eigen::Affine3d &pose,
                       Eigen::Matrix<double, 6, 1> &vel,
                       double &yaw_unwrpd, bool is_base );

    void updatePose( const radar_slam::CarData &msg );
    void updateMap( const radar_ros_interface::RadarData &msg );

    void computeSensorFOV( const Eigen::Affine3d &tf_sensor_to_world,
                           int &start_ind_i, int &end_ind_i, int &start_ind_j, int &end_ind_j );

    void inverseSensorModel( const std::vector<radar_ros_interface::RadarTarget> &targets,
                             const Eigen::Affine3d &tf_sensor_to_world );

    inline void grid_ind_to_world_pos( const int &i, const int &j, Eigen::Vector3d &pos )
    {
        pos = Eigen::Vector3d( ( i + 0.5 ) * map_grid_res + map_origin_( 0 ),
                               ( j + 0.5 ) * map_grid_res + map_origin_( 1 ),
                               0.0 );
    }

    inline void world_pos_to_grid_ind( const Eigen::Vector3d &pos, int &i, int &j )
    {
        i = static_cast<int>( ( pos( 0 ) - map_origin_( 0 ) ) / map_grid_res );
        j = static_cast<int>( ( pos( 1 ) - map_origin_( 1 ) ) / map_grid_res );
    }

    //Normalize to [-180,180):
    inline double constrainAngle( double x )
    {
        x = fmod( x + M_PI, 2.0 * M_PI );
        if( x < 0 )
            x += 2.0 * M_PI;
        return x - M_PI;
    }
    // convert to [-360,360]
    inline double angleConv( double angle )
    {
        return fmod( constrainAngle( angle ), 2.0 * M_PI );
    }
    inline double angleDiff( double a, double b )
    {
        double dif = fmod( b - a + M_PI, 2.0 * M_PI );
        if( dif < 0 )
            dif += 2.0 * M_PI;
        return dif - M_PI;
    }
    inline double unwrap( double previousAngle, double newAngle )
    {
        return previousAngle - angleDiff( newAngle, angleConv( previousAngle ) );
    }

    inline double log_odds_to_prob( double log_odds )
    {
        return ( 1.0 - ( 1.0 / ( 1.0 + exp( log_odds ) ) ) );
    }

    inline int log_odds_to_map_prob( double log_odds )
    {
        return static_cast<int>( 100.0
                * ( 1.0 - ( 1.0 / ( 1.0 + exp( log_odds ) ) ) ) );
    }

    inline double prob_to_log_odds( double prob )
    {
        return log( prob / ( 1.0 - prob ) );
    }

    inline double map_prob_to_log_odds( int prob )
    {
        return log(
                ( static_cast<double>( prob ) / 100.0 ) / ( 1.0
                        - ( static_cast<double>( prob ) / 100.0 ) ) );
    }

    inline double normal_dist( double mu, double var, double x )
    {
        return ( 1.0 / sqrt( 2.0 * M_PI * var ) ) * exp( ( -1.0 / var ) * pow( ( x - mu ), 2.0 ) );
    }

    inline double normal_dist( Eigen::Vector2d mu, Eigen::Matrix2d var, Eigen::Vector2d x )
    {
        std::cout << "mu: " << mu.transpose() << std::endl;
        std::cout << "var: " << var << std::endl;
        std::cout << "x: " << x.transpose() << std::endl;

        return ( 1.0 / ( 2.0 * M_PI * sqrt( var.determinant() ) ) ) * exp(
                -0.5 * ( x - mu ).transpose() * var.inverse() * ( x - mu ) );
    }

    inline double prob_detection( double r_cell, double th_cell, double r_target, double th_target )
    {
    return normal_dist( Eigen::Vector2d( r_target, th_target ),
                Eigen::Vector2d( pow( sigma_r, 2.0 ), pow( sigma_th, 2.0 ) ).asDiagonal(),
                Eigen::Vector2d( r_cell, th_cell ) );
    }

    inline double prob_no_detection( double r_cell, double th_cell, double r_target,
                                     double th_target )
    {
        return normal_dist(
                Eigen::Vector2d( 0.0, th_target ),
                Eigen::Vector2d( pow( sigma_no_r, 2.0 ), pow( sigma_no_th, 2.0 ) ).asDiagonal(),
                Eigen::Vector2d( r_cell, th_cell ) );
    }

    void low_var_resamp( std::vector<double> &weights,
                         std::vector<Eigen::Affine3d> &particles )
    {
        /*        std::cout << "weights:" << std::endl;
        for( auto it = weights.begin(); it != weights.end(); ++it )
        {
            std::cout << *it << " ";
        }
         std::cout << std::endl;*/

        // Create a cumulative sum of the particle weights:
        std::vector<double> weights_cumsum = weights;
        std::partial_sum( weights_cumsum.begin(), weights_cumsum.end(), weights_cumsum.begin() );
        double W = weights_cumsum.back();

        /*        std::cout << "weights_cumsum:" << std::endl;
        for( auto it = weights_cumsum.begin(); it != weights_cumsum.end(); ++it )
        {
            std::cout << *it << " ";
        }
         std::cout << std::endl;*/

        // Draw a single sample from uniform(0,W):
        dist_weights_ = std::uniform_real_distribution<double>( 0.0, W );
        double r = dist_weights_( dist_gen_ );

        // Resample the particles:
        std::vector<Eigen::Affine3d> particles_old = particles;
        int p_ind = ( std::lower_bound( weights_cumsum.begin(), weights_cumsum.end(), r )
                - weights_cumsum.begin() );

        //std::cout << "r, p_ind:" << std::endl;
        //std::cout << r << " " << p_ind << std::endl;
        particles.at( 0 ) = particles_old.at( p_ind );
        for( int i = 1; i < num_particles; ++i )
        {
            p_ind = ( std::lower_bound( weights_cumsum.begin(), weights_cumsum.end(),
                                        fmod( r + ( i * ( W / num_particles ) ), W ) )
                      - weights_cumsum.begin() );
            //std::cout << fmod( r + ( i * ( W / num_particles ) ), W ) << " " << p_ind << std::endl;
            particles.at( i ) = particles_old.at( p_ind );
        }

        // Renormalize the weights:
        double sum = std::accumulate( weights.begin(), weights.end(), 0.0 );
        std::transform( weights.begin(), weights.end(), weights.begin(),
                        std::bind2nd( std::divides<double>(), sum ) );
    }

    double wheelbase;
    double steering_ratio;
    int map_grid_size;
    double map_grid_res;
    int num_update_cells;
    double min_cell_dist;

    double sigma_speed;
    double sigma_steer_angle;
    double rel_speed_thresh;

    double sigma_r;
    double sigma_th;
    double sigma_no_r;
    double sigma_no_th;
    double max_r;
    double max_th;
    double min_snr;
    double max_snr;
    double prob_fa;

    double sigma_px_init;
    double sigma_py_init;
    double sigma_yaw_init;

    int num_particles;
    double p_alpha;
    double w_min_max_thresh;

    int dec_rate;
    bool map_detections_only;
    bool use_sensor_fov;
    bool pf_update_on;

    bool print_debug;

    std::vector<std::string> radar_data_topics;

private:
    Eigen::Affine3d base_pose_;
    double base_yaw_;
    Eigen::Matrix<double, 6, 1> base_vel_;

    std::vector<Eigen::Affine3d> particle_pose_;
    std::vector<double> particle_yaw_;
    std::vector<Eigen::Matrix<double, 6, 1>> particle_vel_;

    std::vector<double> weights_;
    geometry_msgs::PoseArray particle_pose_array_;

    ros::Publisher pub_poses_;

    int dec_ind_;
    ros::Time time_prev_;
    double dt_;
    bool first_time_;

    ros::NodeHandle node_handle_;
    ros::Subscriber sub_car_speed_;
    std::vector<ros::Subscriber> sub_radar_targets_;
    radar_ros_interface::RadarData radar_data_;

    tf2_ros::TransformListener listen_tf_;
    tf2_ros::Buffer buffer_tf_;

    tf2_ros::TransformBroadcaster bcast_tf_;
    geometry_msgs::TransformStamped pose_tf_;
    ros::Publisher pub_ogm_;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> map_log_odds_;
    Eigen::Vector3d map_origin_;
    nav_msgs::OccupancyGrid msg_map_;
    std::default_random_engine dist_gen_;
    std::normal_distribution<double> dist_speed_;
    std::normal_distribution<double> dist_steer_angle_;
    std::normal_distribution<double> dist_px_;
    std::normal_distribution<double> dist_py_;
    std::normal_distribution<double> dist_yaw_;
    std::uniform_real_distribution<double> dist_weights_;

};

#endif
