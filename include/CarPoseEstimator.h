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

/**
 *
 */
class CarPoseEstimator
{
public:
    CarPoseEstimator();
    ~CarPoseEstimator()
    {
    }

    /**
     * Updates a particle's state through the process model.
     *
     * Update the pose/yaw and velocity of a particle using the simple kinematic car model and the
     * car data read from the Panda dongle. Adds a prescribed amount of noise to the car data in
     * order to maintain diversity in the particles, unless the base is being updated directly.
     *
     * @param speed Average wheel speed, as read by the Panda dongle
     * @param steer_angle Steering wheel angle, as read by the Panda dongle
     * @param dt Timestep (time between updates) for integration
     * @param pose The 6d position + orientation of the particle
     * @param vel 6d velocity vector of the particle (linear and rotational velocity)
     * @param yaw_unwrpd Particle yaw angle in radians, unwrapped to be continuous
     * @param is_base Specifies whether a particle or the base is being updated
     */
    void processModel( double speed, double steer_angle, double dt, Eigen::Affine3d &pose,
                       Eigen::Matrix<double, 6, 1> &vel,
                       double &yaw_unwrpd, bool is_base );
    /**
     * Callback for updating the pose of the car using odometric data.
     *
     * Called on receiving a CarData message from a node in communication with the Panda dongle.
     * Uses the car data to update the poses of all particles using the process model.
     *
     * @param msg The CarData message received from the node communicating to the Panda.
     */
    void updatePose( const radar_slam::CarData &msg );

    /**
     * Callback for updating the local grid map and estimated base pose of the car from radar data.
     *
     * Called on receiving a RadarData message from a node in communication with the car radars.
     * Uses the radar data to estimate the likelihood of each particle based on the current grid
     * map, the particle pose and an inverse sensor model, combines all particles into an estimate
     * of the current car pose, and updates the local grid map using the estimate and radar data.
     *
     * @param msg The RadarData message received from a node communcating to a radar sensor.
     */
    void updateMap( const radar_ros_interface::RadarData &msg );

    /**
     * Computes the grid map indices which bound the radar's field of view.
     *
     * Computes the grid map indices which correspond to the "corners" of the radar's field of view,
     * using some approximation to the conical FOV (here, a diamond shape).
     *
     * @param tf_sensor_to_world The pose of the radar sensor in the world frame.
     * @param start_ind_i The start index for dimension "i"
     * @param end_ind_i The end index for dimension "i"
     * @param start_ind_j The start index for dimension "j"
     * @param end_ind_j The end index for dimension "j"
     */
    void computeSensorFOV( const Eigen::Affine3d &tf_sensor_to_world,
                           int &start_ind_i, int &end_ind_i, int &start_ind_j, int &end_ind_j );

    /**
     * Computes the likelihood of grid map occupancy for each radar target.
     *
     * Given an array of radar targets from a sensor with a given pose relative to the world frame,
     * computes the likelihood of occupancy of grid map cells based on a so-called "inverse sensor
     * model" which is chosen/tuned to capture the uncertainty of detections by the sensor.
     *
     * @param targets An array of RadarTarget objects originating from a sensor.
     * @param tf_sensor_to_world The pose in world frame of the sensor.
     */
    void inverseSensorModel( const std::vector<radar_ros_interface::RadarTarget> &targets,
                             const Eigen::Affine3d &tf_sensor_to_world );

    /**
     * Computes the world frame position of a grid map index.
     *
     * @param i The input grid map index "i"
     * @param j The input grid map index "j"
     * @param pos The output world frame position of the input map indices.
     */
    inline void grid_ind_to_world_pos( const int &i, const int &j, Eigen::Vector3d &pos )
    {
        pos = Eigen::Vector3d( ( i + 0.5 ) * map_grid_res + map_origin_( 0 ),
                               ( j + 0.5 ) * map_grid_res + map_origin_( 1 ),
                               0.0 );
    }

    /**
     * Computes the nearest grid map indices to the given world frame position.
     *
     * @param pos The input world frame position to be mapped to grid map indices.
     * @param i The output grid map index "i"
     * @param j The output grid map index "j"
     */
    inline void world_pos_to_grid_ind( const Eigen::Vector3d &pos, int &i, int &j )
    {
        i = static_cast<int>( ( pos( 0 ) - map_origin_( 0 ) ) / map_grid_res );
        j = static_cast<int>( ( pos( 1 ) - map_origin_( 1 ) ) / map_grid_res );
    }

    /**
     * Normalizes an angle to [-PI,PI).
     *
     * @param x The input angle to be constrained to the range
     * @return The output constrained angle
     */
    inline double constrainAngle( double x )
    {
        x = fmod( x + M_PI, 2.0 * M_PI );
        if( x < 0 )
            x += 2.0 * M_PI;
        return x - M_PI;
    }

    /**
     *
     * Converts an angle to [-2*PI,2*PI].
     *
     * @param angle The input angle to be converted
     * @return The output converted angle
     */
    inline double angleConv( double angle )
    {
        return fmod( constrainAngle( angle ), 2.0 * M_PI );
    }

    /**
     * Computes the difference between successive angles a and b.
     *
     * @param a The first angle, in radians
     * @param b The second angle, in radians
     * @return The difference between angles, in radians
     */
    inline double angleDiff( double a, double b )
    {
        double dif = fmod( b - a + M_PI, 2.0 * M_PI );
        if( dif < 0 )
            dif += 2.0 * M_PI;
        return dif - M_PI;
    }

    /**
     * Unwraps an angle using its previous and new values.
     *
     * @param previousAngle Previous unwrapped angle, in radians
     * @param newAngle New wrapped angle, in radians
     * @return The new unwrapped angle, in radians
     */
    inline double unwrap( double previousAngle, double newAngle )
    {
        return previousAngle - angleDiff( newAngle, angleConv( previousAngle ) );
    }

    /**
     * Converts from log-odds representation to probability.
     *
     * @param log_odds Input log-odds representation value
     * @return Output corresponding probability in [0.0, 1.0]
     */
    inline double log_odds_to_prob( double log_odds )
    {
        return ( 1.0 - ( 1.0 / ( 1.0 + exp( log_odds ) ) ) );
    }

    /**
     * Converts from log-odds representation to map probability.
     *
     * Converts a continuous log-odds value to a discrete "map probability" value in [0,100] with
     * 100 corresponding to Prob = 1.0.
     *
     * @param log_odds Input log-odds representation value.
     * @return Output map probability in [0,100]
     */
    inline int log_odds_to_map_prob( double log_odds )
    {
        return static_cast<int>( 100.0
                * ( 1.0 - ( 1.0 / ( 1.0 + exp( log_odds ) ) ) ) );
    }

    /**
     * Converts probability to log-odds representation.
     *
     * @param prob Input probability in [0.0, 1.0]
     * @return Output log-odds representation value
     */
    inline double prob_to_log_odds( double prob )
    {
        return log( prob / ( 1.0 - prob ) );
    }

    /**
     * Converts from map probability to log-odds representation.
     *
     * Converts a discrete map probability in [0,100] to continuous log-odds representation.
     *
     * @param prob Input probability in [0,100]
     * @return Output log-odds representation value
     */
    inline double map_prob_to_log_odds( int prob )
    {
        return log(
                ( static_cast<double>( prob ) / 100.0 ) / ( 1.0
                        - ( static_cast<double>( prob ) / 100.0 ) ) );
    }

    /**
     * Evaluates the probability density function of a univariate normal distribution at a point.
     *
     * @param mu Mean of the normal distribution
     * @param var Variance of the normal distribution
     * @param x Input (query) point value
     * @return Output likelihood of the input point
     */
    inline double normal_dist( double mu, double var, double x )
    {
        return ( 1.0 / sqrt( 2.0 * M_PI * var ) ) * exp( ( -1.0 / var ) * pow( ( x - mu ), 2.0 ) );
    }

    /**
     * Evaluates the probability density function of a bivariate normal distribution at a point.
     *
     * @param mu Mean of the normal distribution
     * @param var Variance of the normal distribution
     * @param x Input (query) point value
     * @return Output likelihood of the input point
     */
    inline double normal_dist( Eigen::Vector2d mu, Eigen::Matrix2d var, Eigen::Vector2d x )
    {
        return ( 1.0 / ( 2.0 * M_PI * sqrt( var.determinant() ) ) ) * exp(
                -0.5 * ( x - mu ).transpose() * var.inverse() * ( x - mu ) );
    }

    /**
     * Computes probability of detection for a grid cell corresponding to a radar measurement.
     *
     * Implements the detection portion of the inverse sensor model for the radar sensor, ie returns
     * the probability of detection for a particular cell given a measured radar target at a given
     * location (both in polar coordinates).  This is assumed to be a Gaussian centered on the
     * target having uncertainty according to the radar sensor's known resolution.
     *
     * @param r_cell The range of the center of the cell, in meters
     * @param th_cell The angle of the center of the cell, in radians
     * @param r_target The range of the radar target, in meters
     * @param th_target The angle of the center of the cell, in radians
     * @return The output probability of detection for the cell in log-odds form
     */
    inline double prob_detection( double r_cell, double th_cell, double r_target, double th_target )
    {
    return normal_dist( Eigen::Vector2d( r_target, th_target ),
                Eigen::Vector2d( pow( sigma_r, 2.0 ), pow( sigma_th, 2.0 ) ).asDiagonal(),
                Eigen::Vector2d( r_cell, th_cell ) );
    }

    /**
     * Computes probability of no detection for a grid cell corresponding to a radar measurement.
     *
     * Implements the no detection portion of the inverse sensor model for the radar sensor, ie
     * returns the probability of not detecting an object in a particular cell given a measured
     * radar target at a given location (both in polar coordinates).  This is assumed to be a
     * Gaussian centered on the radar sensor having uncertainty based on heuristics.
     *
     * @param r_cell The range of the center of the cell, in meters
     * @param th_cell The angle of the center of the cell, in radians
     * @param r_target The range of the radar target, in meters
     * @param th_target The angle of the center of the cell, in radians
     * @return The output probability of detection for the cell in log-odds form
     */
    inline double prob_no_detection( double r_cell, double th_cell, double r_target,
                                     double th_target )
    {
        return normal_dist(
                Eigen::Vector2d( 0.0, th_target ),
                Eigen::Vector2d( pow( sigma_no_r, 2.0 ), pow( sigma_no_th, 2.0 ) ).asDiagonal(),
                Eigen::Vector2d( r_cell, th_cell ) );
    }

    /**
     * Low-variance particle filtering resampling algorithm.
     *
     * Resamples the particles of a particle filter according to the so-called "Low Variance"
     * resampling algorithm, which is a simple but effective resampling strategy which helps avoid
     * particle deficiency (ie all particles converging to one location over time).
     *
     * @param weights Vector of particle weights, each in [0.0, 1.0] and summing to 1.0
     * @param particles Vector of particle pose hypotheses
     */
    void low_var_resamp( std::vector<double> &weights,
                         std::vector<Eigen::Affine3d> &particles )
    {
        // Create a cumulative sum of the particle weights:
        std::vector<double> weights_cumsum = weights;
        std::partial_sum( weights_cumsum.begin(), weights_cumsum.end(), weights_cumsum.begin() );
        double W = weights_cumsum.back();

        // Draw a single sample from uniform(0,W):
        dist_weights_ = std::uniform_real_distribution<double>( 0.0, W );
        double r = dist_weights_( dist_gen_ );

        // Resample the particles:
        std::vector<Eigen::Affine3d> particles_old = particles;
        int p_ind = ( std::lower_bound( weights_cumsum.begin(), weights_cumsum.end(), r )
                - weights_cumsum.begin() );

        particles.at( 0 ) = particles_old.at( p_ind );
        for( int i = 1; i < num_particles; ++i )
        {
            p_ind = ( std::lower_bound( weights_cumsum.begin(), weights_cumsum.end(),
                                        fmod( r + ( i * ( W / num_particles ) ), W ) )
                      - weights_cumsum.begin() );
            particles.at( i ) = particles_old.at( p_ind );
        }

        // Renormalize the weights:
        double sum = std::accumulate( weights.begin(), weights.end(), 0.0 );
        std::transform( weights.begin(), weights.end(), weights.begin(),
                        std::bind2nd( std::divides<double>(), sum ) );
    }

    // Parameters loaded from YAML file:
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
