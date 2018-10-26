#include "CarPoseEstimator.h"
#include <numeric>

CarPoseEstimator::CarPoseEstimator( void ) :
        listen_tf_( buffer_tf_ ),
        dist_speed_( 0.0, 1.0 ),
        dist_steer_angle_( 0.0, 1.0 ),
        dist_px_( 0.0, 1.0 ),
        dist_py_( 0.0, 1.0 ),
        dist_yaw_( 0.0, 1.0 ),
        dist_weights_( 0.0, 1.0 ),
        first_time_( true )
{
    // Load the YAML parameters:
    node_handle_.getParam( "wheelbase", wheelbase );
    node_handle_.getParam( "steering_ratio", steering_ratio );

    node_handle_.getParam( "map_grid_res", map_grid_res );
    node_handle_.getParam( "map_grid_size", map_grid_size );
    node_handle_.getParam( "num_update_cells", num_update_cells );
    node_handle_.getParam( "min_cell_dist", min_cell_dist );

    node_handle_.getParam( "sigma_r", sigma_r );
    node_handle_.getParam( "sigma_th", sigma_th );
    node_handle_.getParam( "sigma_no_r", sigma_no_r );
    node_handle_.getParam( "sigma_no_th", sigma_no_th );
    node_handle_.getParam( "max_r", max_r );
    node_handle_.getParam( "max_th", max_th );
    node_handle_.getParam( "min_snr", min_snr );
    node_handle_.getParam( "max_snr", max_snr );
    node_handle_.getParam( "prob_fa", prob_fa );

    node_handle_.getParam( "sigma_speed", sigma_speed );
    node_handle_.getParam( "sigma_steer_angle", sigma_steer_angle );

    node_handle_.getParam( "sigma_px_init", sigma_px_init );
    node_handle_.getParam( "sigma_py_init", sigma_py_init );
    node_handle_.getParam( "sigma_yaw_init", sigma_yaw_init );

    node_handle_.getParam( "num_particles", num_particles );
    node_handle_.getParam( "p_alpha", p_alpha );
    node_handle_.getParam( "w_min_max_thresh", w_min_max_thresh );

    node_handle_.getParam( "dec_rate", dec_rate );
    node_handle_.getParam( "map_detections_only", map_detections_only );
    node_handle_.getParam( "use_sensor_fov", use_sensor_fov );
    node_handle_.getParam( "pf_update_on", pf_update_on );

    node_handle_.getParam( "print_debug", print_debug );

    node_handle_.getParam( "radar_data_topics", radar_data_topics );

    // Initialize car position and orientation (pose):
    base_pose_.translation() = Eigen::Vector3d::Zero();
    base_pose_.linear() =
            Eigen::AngleAxisd( 0.0 * M_PI, Eigen::Vector3d::UnitZ() ).toRotationMatrix();

    // Initialize the base pose particles and weights:
    particle_pose_.resize( num_particles );
    particle_vel_.resize( num_particles );
    particle_yaw_.resize( num_particles );
    weights_.resize( num_particles, ( 1.0 / static_cast<double>( num_particles ) ) );

    // Initialize motion (process) model noise params:
    dist_speed_ = std::normal_distribution<double>( 0.0, sigma_speed );
    dist_steer_angle_ = std::normal_distribution<double>( 0.0, sigma_steer_angle );

    // Sample the particles:
    dist_px_ = std::normal_distribution<double>( 0.0, sigma_px_init );
    dist_py_ = std::normal_distribution<double>( 0.0, sigma_py_init );
    dist_yaw_ = std::normal_distribution<double>( 0.0, sigma_yaw_init );

    for( int i = 0; i < num_particles; ++i )
    {
        particle_pose_.at( i ).translation() = base_pose_.translation()
                + Eigen::Vector3d( dist_px_( dist_gen_ ), dist_py_( dist_gen_ ), 0.0 );
        particle_pose_.at( i ).linear() =
                Eigen::AngleAxisd( dist_yaw_( dist_gen_ ), Eigen::Vector3d::UnitZ() ).toRotationMatrix() * base_pose_.linear();
        particle_yaw_.at( i ) = particle_pose_.at( i ).linear().eulerAngles( 0, 1, 2 )( 2 );
    }

    // Initialize particle pose array info:
    particle_pose_array_.header.frame_id = "/base_link";
    particle_pose_array_.poses.resize( num_particles );
    pub_poses_ = node_handle_.advertise<geometry_msgs::PoseArray>( "particle_poses", 10 );

    // Subscribe to the car_speed topic with the updatePose callback:
    sub_car_speed_ = node_handle_.subscribe( "car_data", 10,
                                             &CarPoseEstimator::updatePose,
                                             this );

    // Subscribe to the radar target topics with the updateMap callback:
    sub_radar_targets_.resize( radar_data_topics.size() );
    for( int i = 0; i < radar_data_topics.size(); ++i )
    {
        sub_radar_targets_.at( i ) = node_handle_.subscribe(
                radar_data_topics.at( i ), 10,
                                                             &CarPoseEstimator::updateMap, this );
    }

    //Initialize the log-odds map representation:
    map_log_odds_ = Eigen::MatrixXd::Zero( map_grid_size, map_grid_size );
    map_origin_ = Eigen::Vector3d(
            -0.5 * map_grid_size * map_grid_res,
                                   -0.5 * map_grid_size * map_grid_res, 0.0 );


    // Initialize and advertise the grid map message:
    msg_map_.data = std::vector<int8_t>(
            map_grid_size * map_grid_size );
    msg_map_.info.map_load_time = ros::Time::now();
    msg_map_.info.width = map_grid_size;
    msg_map_.info.height = map_grid_size;
    msg_map_.info.resolution = map_grid_res;
    msg_map_.header.frame_id = "base_link";
    msg_map_.info.origin.position.x = map_origin_( 0 );
    msg_map_.info.origin.position.y = map_origin_( 1 );
    msg_map_.info.origin.position.z = map_origin_( 2 );

    pub_ogm_ = node_handle_.advertise<nav_msgs::OccupancyGrid>( "grid_map",
                                                                10 );

    dec_ind_ = 0;
}

void CarPoseEstimator::computeSensorFOV( const Eigen::Affine3d &tf_sensor_to_world,
                                         int &start_ind_i, int &end_ind_i, int &start_ind_j,
                                         int &end_ind_j )
{
    // Compute the corners of the sensor FOV cone (using a diamond approximation):
    std::array<Eigen::Vector3d, 4> fov_corners; // clockwise from sensor origin
    double rc = max_r * cos( max_th );
    double rs = max_r * sin( max_th );
    fov_corners.at( 0 ) = tf_sensor_to_world.translation();
    fov_corners.at( 1 ) = tf_sensor_to_world
            * Eigen::Vector3d( rc, -rs, 0.0 );
    fov_corners.at( 2 ) = tf_sensor_to_world * Eigen::Vector3d( max_r, 0.0, 0.0 );
    fov_corners.at( 3 ) = tf_sensor_to_world
            * Eigen::Vector3d( rc, rs, 0.0 );

    // Find the min and max indices for the corners (possibly speed this up with c++11 functions):
    int i, j;
    start_ind_i = map_grid_size;
    end_ind_i = 0;
    start_ind_j = map_grid_size;
    end_ind_j = 0;
    for( auto it = fov_corners.begin(); it != fov_corners.end(); ++it )
    {
        world_pos_to_grid_ind( *it, i, j );
        start_ind_i = std::min( i, start_ind_i );
        end_ind_i = std::max( i, end_ind_i );
        start_ind_j = std::min( j, start_ind_j );
        end_ind_j = std::max( j, end_ind_j );
    }
}

void CarPoseEstimator::inverseSensorModel(
        const std::vector<radar_ros_interface::RadarTarget> &targets,
        const Eigen::Affine3d &tf_sensor_to_world )
{
    if( pf_update_on )
    {
        // Compute particle weights from current measurements:
        Eigen::Vector3d p_sensor, p_world;
        int target_i, target_j;
        double p_weight;
        for( int i = 0; i < num_particles; ++i )
        {
            p_weight = 0.0;
            for( auto it = targets.begin(); it != targets.end(); ++it )
            {
                if( ( it->range < max_r ) && ( it->snr >= min_snr ) )
                {
                    //std::cout << it->speed << std::endl;
                    // Compute position of the target in the sensor frame:
                    p_sensor( 0 ) = cos( ( M_PI / 180.0 ) * it->azimuth )
                            * cos( ( M_PI / 180.0 ) * it->elevation ) * it->range;
                    p_sensor( 1 ) = sin( ( M_PI / 180.0 ) * it->azimuth )
                            * cos( ( M_PI / 180.0 ) * it->elevation ) * it->range;
                    p_sensor( 2 ) = sin( ( M_PI / 180.0 ) * it->elevation ) * it->range;

                    // Transform the target position to the world frame using the particle pose:
                    p_world = particle_pose_.at( i ) * base_pose_.inverse() * tf_sensor_to_world
                              * p_sensor;
                    world_pos_to_grid_ind( p_world, target_i, target_j );

                    // Get the probability at the particle's target position and update particle weight:
                    p_weight += log_odds_to_prob(
                            map_log_odds_( target_i, target_j ) );
                }
            }

            // Low-pass filter the weight update:
            weights_.at( i ) = ( 1.0 - p_alpha ) * p_weight + p_alpha * weights_.at( i );
        }

        // Renormalize particle weights:
        double sum = std::accumulate( weights_.begin(), weights_.end(), 0.0 );
        std::transform( weights_.begin(), weights_.end(), weights_.begin(),
                        std::bind2nd( std::divides<double>(), sum ) );

        // Print the particles and their weights:
        if( print_debug )
        {
            std::cout << "particles: " << std::endl;
            for( auto it = particle_pose_.begin(); it != particle_pose_.end(); ++it )
            {
                std::cout << "pos: " << it->translation().transpose() << "yaw: " << (it->linear().eulerAngles(0,1,2))(2) << std::endl;
            }
            std::cout << std::endl;
            std::cout << "weights: ";
            for( auto it = weights_.begin(); it != weights_.end(); ++it )
            {
                std::cout << *it << " ";
            }
            std::cout << std::endl;
        }

        // Compute the estimated pose from the particles:
        base_yaw_ = 0.0;
        Eigen::Vector3d rpy;
        base_pose_ = Eigen::Affine3d::Identity();
        for( int i = 0; i < num_particles; ++i )
        {
            base_pose_.translation() += weights_.at( i ) * particle_pose_.at( i ).translation();
            base_yaw_ += weights_.at( i ) * particle_yaw_.at( i );
        }
        base_pose_.linear() =
                Eigen::AngleAxisd( base_yaw_, Eigen::Vector3d::UnitZ() ).toRotationMatrix();

        // Resample the particles if necessary to prevent degeneracy:
        if( ( *std::min_element( weights_.begin(), weights_.end() ) ) / ( *std::max_element(
                weights_.begin(), weights_.end() ) )
            < w_min_max_thresh )
        {
            std::cout << "RESAMPLING" << std::endl;
            low_var_resamp( weights_, particle_pose_ );
        }
    }

    // Update the map using the new measurements and updated pose from the particle filter:
    int start_ind_i, end_ind_i, start_ind_j, end_ind_j;
    if( use_sensor_fov )
    {
        // Only update map in the current sensor direction:
        computeSensorFOV( tf_sensor_to_world, start_ind_i, end_ind_i, start_ind_j, end_ind_j );
    }
    else
    {
        // Update map in all directions around current car position:
        // Get the cell indices of the current car position:
        int car_ind_i, car_ind_j;
        world_pos_to_grid_ind( base_pose_.translation(), car_ind_i, car_ind_j );

        start_ind_i = car_ind_i - 0.5 * num_update_cells;
        end_ind_i = car_ind_i + 0.5 * num_update_cells;
        start_ind_j = car_ind_j - 0.5 * num_update_cells;
        end_ind_j = car_ind_j + 0.5 * num_update_cells;
    }

    // Update the map using an inverse sensor model:
    for( int i = start_ind_i; i < end_ind_i; ++i )
    {
        for( int j = start_ind_j; j < end_ind_j; ++j )
        {
            // Get the position of the center of the cell in world frame:
            Eigen::Vector3d p_cell_world;
            grid_ind_to_world_pos( i, j, p_cell_world );

            // Transform into sensor frame:
            Eigen::Vector3d p_cell_sensor = tf_sensor_to_world.inverse() * p_cell_world;

            // Convert to polar coordinates to compare to target:
            double r = p_cell_sensor.norm();
            double th = atan2( p_cell_sensor( 1 ), p_cell_sensor( 0 ) );
            double p_d;

            if( r <= max_r && std::abs( th ) <= max_th )
            {
                //std::cout << "r: " << r << "th: " << th << std::endl;
                for( auto it = targets.begin(); it != targets.end(); ++it )
                {
                    /*                    std::cout << "target r: " << it->range << "target th: "
                              << ( M_PI / 180.0 ) * it->azimuth
                     << std::endl;*/

                    /*                    p_d = std::pow( prob_fa, ( 1.0 / ( 1.0 + it->snr ) ) );
                    //std::cout << "p_d: " << p_d << std::endl;
                    map_log_odds_( i, j ) += prob_to_log_odds(
                            p_d * prob_detection( r, th, it->range,
                                                  ( M_PI / 180.0 ) * it->azimuth ) );
                    map_log_odds_( i, j ) -= prob_to_log_odds(
                            p_d * prob_no_detection( r, th, it->range,
                                                     ( M_PI / 180.0 ) * it->azimuth ) );
                    std::cout << "prob detection: "
                    << prob_detection( r, th, it->range, ( M_PI / 180.0 ) * it->azimuth )
                    << "prob no detection: "
                    << prob_no_detection( r, th, it->range, ( M_PI / 180.0 ) * it->azimuth )
                     << std::endl;*/

                    p_d = std::pow( prob_fa, ( 1.0 / ( 1.0 + it->snr ) ) );
                    map_log_odds_( i, j ) += p_d
                            * normal_dist( it->range, sigma_r, r )
                                            * normal_dist(
                                    ( M_PI / 180.0 ) * it->azimuth, sigma_th,
                                                           th );

                    map_log_odds_( i, j ) -= p_d
                            * normal_dist( 0.0, 0.5 * it->range, r )
                            * normal_dist( ( M_PI / 180.0 ) * it->azimuth, sigma_th,
                     th );
                }
            }

            msg_map_.data.at( j * map_grid_size + i ) = log_odds_to_map_prob(
                    map_log_odds_( i, j ) );
        }
    }

    // Publish the particle pose array:
    particle_pose_array_.header.stamp = ros::Time::now();
    for( int i = 0; i < num_particles; ++i )
    {
        particle_pose_array_.poses.at( i ) = tf2::toMsg( particle_pose_.at( i ) );
    }
    pub_poses_.publish( particle_pose_array_ );
}

void CarPoseEstimator::processModel( double speed, double steer_angle, double dt,
                                     Eigen::Affine3d &pose, Eigen::Matrix<double, 6, 1> &vel,
                                     double &yaw_unwrpd )
{
    // Add noise to the inputs:
    speed += dist_speed_( dist_gen_ );
    steer_angle += dist_steer_angle_( dist_gen_ );

    // Update the orientation using speed and steering angle:
    vel.segment( 3, 3 ) = Eigen::Vector3d(
            0.0, 0.0,
            ( speed / wheelbase )
            * tan( ( M_PI / 180.0 ) * ( steer_angle / steering_ratio ) ) );
    pose.linear() = Eigen::AngleAxisd( dt * vel( 5 ), Eigen::Vector3d::UnitZ() ).toRotationMatrix()
            * pose.linear();

    // Update position using speed and updated orientation:
    yaw_unwrpd = unwrap( yaw_unwrpd, pose.linear().eulerAngles( 0, 1, 2 )( 2 ) );
    vel.segment( 0, 3 ) = Eigen::Vector3d( speed * cos( yaw_unwrpd ), speed * sin( yaw_unwrpd ),
                                           0.0 );
    pose.translation() += dt * vel.segment( 0, 3 );
}

void CarPoseEstimator::updatePose( const radar_slam::CarData &msg )
{
    // Update the timestep, handling first time carefully:
    ros::Time time_now = ros::Time::now();
    if( first_time_ )
    {
        time_prev_ = time_now;
        first_time_ = false;
    }
    dt_ = ( time_now - time_prev_ ).toSec();

    // Update orientation using speed and steering angle:
    if( map_detections_only )
    {
        processModel( msg.speed, msg.steer_angle, dt_, base_pose_, base_vel_, base_yaw_ );
    }
    else
    {
        if( pf_update_on ) // propagate all particles through the process model:
        {
            for( int i = 0; i < num_particles; ++i )
            {
                processModel( msg.speed, msg.steer_angle, dt_, particle_pose_.at( i ),
                              particle_vel_.at( i ),
                              particle_yaw_.at( i ) );
            }
        }
        else // propagate only the base pose through the process model:
        {
            processModel( msg.speed, msg.steer_angle, dt_, base_pose_, base_vel_, base_yaw_ );
        }
    }

    time_prev_ = time_now;
}

void CarPoseEstimator::updateMap( const radar_ros_interface::RadarData &msg )
{
    // Get the data frame ID and look up the corresponding tf transform:
    Eigen::Affine3d tf_sensor_to_world = tf2::transformToEigen(
            buffer_tf_.lookupTransform( "base_link", msg.header.frame_id, ros::Time( 0 ) ) );

    if( map_detections_only )
    {
        // Find the nearest grid cell to each RAW detection and set it occupied:
        Eigen::Vector3d p_sensor, p_world;
        int target_i, target_j;
        std::vector<radar_ros_interface::RadarTarget> targets = msg.raw_targets;
        for( auto it = targets.begin(); it != targets.end(); ++it )
        {
            if( ( it->range < max_r ) && ( it->snr >= min_snr ) )
            {
                p_sensor( 0 ) = cos( ( M_PI / 180.0 ) * it->azimuth )
                        * cos( ( M_PI / 180.0 ) * it->elevation ) * it->range;
                p_sensor( 1 ) = sin( ( M_PI / 180.0 ) * it->azimuth )
                        * cos( ( M_PI / 180.0 ) * it->elevation ) * it->range;
                p_sensor( 2 ) = sin( ( M_PI / 180.0 ) * it->elevation ) * it->range;

                p_world = tf_sensor_to_world * p_sensor;
                world_pos_to_grid_ind( p_world, target_i, target_j );

                map_log_odds_( target_i, target_j ) = 1.0;
                msg_map_.data.at( target_j * map_grid_size + target_i ) =
                        log_odds_to_map_prob(
                                map_log_odds_( target_i, target_j ) );
            }
        }
    }
    else
    {
        if( ( dec_ind_ % dec_rate ) == 0 )
        {
            // Update the log-odds map:
            inverseSensorModel( msg.raw_targets, tf_sensor_to_world );
        }
        ++dec_ind_;
    }

    // Update the map message and publish:
    msg_map_.header.seq = 1;
    msg_map_.header.stamp = ros::Time::now();
    pub_ogm_.publish( msg_map_ );

    // Send the updated car pose transform:
    ros::Time time_now = ros::Time::now();
    pose_tf_ = tf2::eigenToTransform( base_pose_ );
    pose_tf_.header.stamp = time_now;
    pose_tf_.child_frame_id = "chassis";
    pose_tf_.header.frame_id = "base_link";
    bcast_tf_.sendTransform( pose_tf_ );
}
