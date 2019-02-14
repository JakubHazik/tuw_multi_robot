#include <tuw_global_router/single_robot_router_node.h>
#include <tuw_global_router/srr_utils.h>
#include <tuw_multi_robot_msgs/Route.h>
#include <chrono>
#include <boost/functional/hash.hpp>
#include <boost/regex.hpp>
#include <tf/tf.h>
#include <unordered_map>
#include <libssh/libssh.h>

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "single_robot_router" ); /// initializes the ros node with default name
    ros::NodeHandle n;

    ros::Rate r ( 1 );

    multi_robot_router::SingleRobotRouterNode node ( n );

    while ( ros::ok() ) {
        r.sleep();
        ros::spinOnce();
        node.monitorExecution();
    }

    return 0;
}

namespace multi_robot_router {

SingleRobotRouterNode::SingleRobotRouterNode ( ros::NodeHandle &_n ) : Router(),
    n_ ( _n ),
    n_param_ ( "~" ),
    monitor_enabled_ ( false ),
    attempts_total_(0),
    attempts_successful_(0),
    sum_processing_time_total_(.0),
    sum_processing_time_successful_(.0) {
    id_ = 0;

    n_param_.param<std::string> ( "frame_id", frame_id_, frame_id_ );
    n_param_.param<std::string> ( "robot_name", robot_name_, robot_name_ );
    n_param_.param<double> ( "robot_radius", robot_radius_, robot_radius_ );

    // Static subscriptions
    sub_map_ = n_.subscribe ( "map", 1, &SingleRobotRouterNode::mapCallback, this );
    sub_voronoi_graph_ = n_.subscribe ( "segments", 1, &SingleRobotRouterNode::graphCallback, this );
    sub_robot_goal_ = n_.subscribe ( "goal", 1, &SingleRobotRouterNode::goalCallback, this );
    sub_odometry_ = n_.subscribe ( robot_name_ + "/odom", 1, &SingleRobotRouterNode::odometryCallback, this );

    // Static publishers
    pubPlannerStatus_ = n_.advertise<tuw_multi_robot_msgs::RouterStatus> ( "planner_status", 1 );

    // Dynamic reconfigure
    call_type = boost::bind ( &SingleRobotRouterNode::parametersCallback, this, _1, _2 );
    param_server.setCallback ( call_type );

}

void SingleRobotRouterNode::monitorExecution() {

    /*for ( const RobotInfoPtr robot: active_robots_ ) {
        // Check if each active robot has reached its goal
        bool found_goal=false;
        std::vector<tuw_multi_robot_msgs::RobotGoals>::const_iterator goal_it;

        Eigen::Vector2d actual_position{robot->getPose()[0],robot->getPose()[1]}; 
        Eigen::Vector2d goal_position{0.0,0.0};

        for ( auto goal=goals_msg_.robots.begin() ; goal!=goals_msg_.robots.end() ; goal++ ) {
            if(!goal->robot_name.compare(robot->robot_name)) {
                goal_position[0]=goal->destinations[0].position.x;
                goal_position[1]=goal->destinations[0].position.y;
                goal_it = goal; 
                found_goal=true;
                break;
            }
        }

        if(found_goal) {
            if((actual_position-goal_position).norm() < 0.5) {
                goals_msg_.robots.erase(goal_it); 
            }
        }
    }


    ROS_INFO("Number of active robots : %d", goals_msg_.robots.size());*/

}

void SingleRobotRouterNode::goalCallback ( const geometry_msgs::PoseStamped &msg ) {

    goal_=msg; 
    plan();

}


void SingleRobotRouterNode::odometryCallback( const nav_msgs::Odometry::ConstPtr & _odom) {

    // Transform the robot's pose in the global frame and send it to the progress monitor
    geometry_msgs::PoseStamped pose_odom_frame, pose_world_frame;
    pose_odom_frame.header=_odom->header;
    pose_odom_frame.pose=_odom->pose.pose;

    try {
        ros::Time now = ros::Time::now();
        if(tf_listener_.waitForTransform(_odom->header.frame_id, frame_id_, now, ros::Duration(1.0))) {
            tf_listener_.transformPose(frame_id_, pose_odom_frame, pose_world_frame);
        }
        else
        {
            ROS_ERROR("Single robot router : transform between %s and %s is not available",_odom->header.frame_id.c_str(),frame_id_.c_str());
            return;
        }
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    tuw_multi_robot_msgs::RobotInfo robot_info;
    robot_info.header = pose_world_frame.header;
    robot_info.robot_name = robot_name_;
    robot_info.pose.pose = pose_world_frame.pose;
    robot_info.pose.covariance = _odom->pose.covariance;

    auto robot = RobotInfo::findObj ( active_robots_, robot_name_ );
    // If new robot connected, add it to the router
    if ( robot == active_robots_.end() ) {
        // Create new entry
        RobotInfoPtr robot_new = std::make_shared<RobotInfo> ( robot_info );
        robot_new->initTopics ( n_, true );
        active_robots_.push_back ( robot_new );
    } else {
        ( *robot )->updateInfo ( robot_info );
    }

}


void SingleRobotRouterNode::parametersCallback ( tuw_multi_robot_router::routerConfig &config, uint32_t level ) {
    // Important set router before settings
    uint32_t threads = config.nr_threads;
    if ( config.router_type == 1 )
        setPlannerType ( routerType::multiThreadSrr, threads );
    else
        setPlannerType ( routerType::singleThread, 1 );

    if ( config.collision_resolver == 0 ){
        setCollisionResolutionType ( SegmentExpander::CollisionResolverType::none );
        collisionResolver_ = false;
    } else if ( config.collision_resolver == 1 ) {
        setCollisionResolutionType ( SegmentExpander::CollisionResolverType::backtracking );
        collisionResolver_ = true;
    }else{
        setCollisionResolutionType ( SegmentExpander::CollisionResolverType::avoidance );
        collisionResolver_ = true;
    }

    if ( config.voronoi_graph )
        graphMode_ = graphType::voronoi;
    else
        graphMode_ = graphType::random;

    if ( config.goal_mode == 0 )
        goalMode_ = goalMode::use_map_goal;
    else if ( config.goal_mode == 1 )
        goalMode_ = goalMode::use_voronoi_goal;
    else
        goalMode_ = goalMode::use_segment_goal;

    routerTimeLimit_s_ = config.router_time_limit_s;
    topic_timeout_s_ = config.topic_timeout_s;

    priorityRescheduling_ = config.priority_rescheduling;
    speedRescheduling_ = config.speed_rescheduling;
    segmentOptimizations_ = config.path_endpoint_optimization;
    publish_routing_table_ = config.publish_routing_table;
}


void SingleRobotRouterNode::mapCallback ( const nav_msgs::OccupancyGrid &_map ) {
    std::vector<signed char> map = _map.data;

    Eigen::Vector2d origin;
    origin[0] = _map.info.origin.position.x;
    origin[1] = _map.info.origin.position.y;

    size_t new_hash = getHash ( map, origin, _map.info.resolution );

    ROS_INFO ( "map %f %f %f", origin[0], origin[1], _map.info.resolution );

    if ( new_hash != current_map_hash_ ) {
        mapOrigin_[0] = _map.info.origin.position.x;
        mapOrigin_[1] = _map.info.origin.position.y;
        mapResolution_ = _map.info.resolution;

        cv::Mat m ( _map.info.height, _map.info.width, CV_8SC1, map.data() );

        m.convertTo ( m, CV_8UC1 );
        cv::bitwise_not ( m, m );

        cv::threshold ( m, m, 40, 255, CV_THRESH_BINARY | CV_THRESH_OTSU );
        cv::distanceTransform ( m, distMap_, CV_DIST_L1, 3 );

        current_map_hash_ = new_hash;
        got_map_ = true;

        ROS_INFO ( "%s: New Map %i %i %lu", n_param_.getNamespace().c_str() , _map.info.width, _map.info.height, current_map_hash_ );
    }
}


void SingleRobotRouterNode::graphCallback ( const tuw_multi_robot_msgs::Graph &msg ) {
    std::vector<Segment> graph;

    segment_width_.clear();

    for ( const tuw_multi_robot_msgs::Vertex &segment : msg.vertices ) {
        std::vector<Eigen::Vector2d> points;

        for ( const geometry_msgs::Point &point : segment.path ) {
            points.emplace_back ( point.x, point.y );
        }

        std::vector<uint32_t> successors;

        for ( const auto &succ : segment.successors ) {
            successors.emplace_back ( succ );
        }

        std::vector<uint32_t> predecessors;

        for ( const auto &pred : segment.predecessors ) {
            predecessors.emplace_back ( pred );
        }

        if ( segment.valid ) {
            graph.emplace_back ( segment.id, points, successors, predecessors,  2 * robot_radius_ / mapResolution_); 
        } else {
            graph.emplace_back ( segment.id, points, successors, predecessors, 0 );
        }

        segment_width_.emplace_back ( segment.width );
        
    }

    std::sort ( graph.begin(), graph.end(), sortSegments );

    size_t hash = getHash ( graph );

    if ( current_graph_hash_ != hash ) {
        current_graph_hash_ = hash;
        graph_ = graph;
        ROS_INFO ( "%s: Graph %lu", n_param_.getNamespace().c_str() , hash );
    }
    got_graph_ = true;
}


void SingleRobotRouterNode::plan() {

    // Initialize data for single robot
    std::vector<Eigen::Vector3d> starts;
    starts.push_back(Eigen::Vector3d{active_robots_[0]->pose.pose.position.x, 
                                     active_robots_[0]->pose.pose.position.y,
                                     getYaw ( active_robots_[0]->pose.pose.orientation ) } ); 
   
    ROS_ERROR("Start : %f %f", active_robots_[0]->pose.pose.position.x, active_robots_[0]->pose.pose.position.y);
 
    std::vector<Eigen::Vector3d> goals;
    goals.push_back(Eigen::Vector3d{goal_.pose.position.x, 
                                    goal_.pose.position.y,
                                    getYaw ( goal_.pose.orientation ) } ) ; 

    std::vector<float> radius;
    radius.push_back(robot_radius_);

    std::vector<std::string> robot_names;
    robot_names.push_back(robot_name_);

    if ( got_map_ && got_graph_ ) {
        ROS_DEBUG("Multi-robot router : planning preparation successful");
        attempts_total_++;
        // Try to find a plan
        auto t1 = std::chrono::high_resolution_clock::now();
        plan_found_=false;
        plan_found_ = makePlan ( starts, goals, radius, distMap_, mapResolution_, mapOrigin_, graph_, robot_names );
        auto t2 = std::chrono::high_resolution_clock::now();
        int duration = std::chrono::duration_cast<std::chrono::milliseconds> ( t2 - t1 ).count();
        sum_processing_time_total_ += duration;
        if ( plan_found_ ) {
            int nx = distMap_.cols;
            int ny = distMap_.rows;

            double res = mapResolution_;
            int cx = mapOrigin_[0];
            int cy = mapOrigin_[1];

            publish();
            attempts_successful_++;
            sum_processing_time_successful_ += duration;
            freshPlan_ = false;
        } else {
            publishEmpty();
        }
        float rate_success = ((float) attempts_successful_) / (float) attempts_total_;
        float avr_duration_total = sum_processing_time_total_ / (float) attempts_total_;
        float avr_duration_successful = sum_processing_time_successful_ / (float) attempts_successful_;
        ROS_INFO ( "\nSuccess %i, %i = %4.3f, avr %4.0f ms, success: %4.0f ms, %s, %s, %s \n [%4.3f, %4.0f,  %4.0f]", 
              attempts_successful_, attempts_total_,  rate_success, avr_duration_total, avr_duration_successful,
              (priorityRescheduling_?"PR= on":"PR= off"), (speedRescheduling_?"SR= on":"SR= off"), (collisionResolver_?"CR= on":"CR= off"),
              rate_success, avr_duration_total, avr_duration_successful);

        id_++;

    } else if ( !got_map_ || !got_graph_ ) {
        publishEmpty();
        ROS_INFO ( "%s: Multi Robot Router: No Map or Graph received", n_param_.getNamespace().c_str() );
    } else {
        publishEmpty();
    }

}


void SingleRobotRouterNode::publishEmpty() {

    if(publish_routing_table_ == false) return;
    ROS_INFO ( "%s: No Plan found!!!!, publishing empty plan", n_param_.getNamespace().c_str());
    time_first_robot_started_ = ros::Time::now();
    nav_msgs::Path msg_path;
    tuw_multi_robot_msgs::Route msg_route;
    msg_path.header.seq = 0;
    msg_path.header.stamp = time_first_robot_started_;
    msg_path.header.frame_id = frame_id_;
    msg_route.header = msg_path.header;

    for ( RobotInfoPtr &robot: active_robots_ ) {
        robot->pubPaths_.publish ( msg_path );
        robot->pubRoute_.publish ( msg_route );
    }

    mrrp_status_.id = id_;
    mrrp_status_.success = 0;
    mrrp_status_.duration = getDuration_ms();
    pubPlannerStatus_.publish ( mrrp_status_ );
}


void SingleRobotRouterNode::publish() {
    if(publish_routing_table_ == false) return;
    ROS_INFO ( "%s: Plan found :-), publishing plan", n_param_.getNamespace().c_str());
    time_first_robot_started_ = ros::Time::now();
    nav_msgs::Path msg_path;
    tuw_multi_robot_msgs::Route msg_route;
    msg_path.header.seq = 0;
    msg_path.header.stamp = time_first_robot_started_;
    msg_path.header.frame_id = frame_id_;
    msg_route.header = msg_path.header;

    for ( int i = 0; i < active_robots_.size(); i++ ) {
        RobotInfoPtr robot = active_robots_[i];
        const std::vector<Checkpoint> &route = getRoute ( i );
        msg_path.poses.clear();

        //Add first point
        geometry_msgs::PoseStamped pose_1;
        pose_1.header.seq = 0;
        pose_1.header.stamp = time_first_robot_started_;
        pose_1.header.frame_id = frame_id_;

        Eigen::Vector2d pos ( route[0].start[0] * mapResolution_, route[0].start[1] * mapResolution_ );
        pose_1.pose.position.x = pos[0] + mapOrigin_[0];
        pose_1.pose.position.y = pos[1] + mapOrigin_[1];

        pose_1.pose.orientation.w = 1;
        msg_path.poses.push_back ( pose_1 );

        //Add other points
        for ( const Checkpoint &c : route ) {
            geometry_msgs::PoseStamped pose;
            pose.header.seq = 0;
            pose.header.stamp = time_first_robot_started_;
            pose.header.frame_id = frame_id_;

            Eigen::Vector2d pos ( c.end[0] * mapResolution_, c.end[1] * mapResolution_ );
            pose.pose.position.x = pos[0] + mapOrigin_[0];
            pose.pose.position.y = pos[1] + mapOrigin_[1];

            tf::Quaternion q;
            q.setEuler ( 0, 0, c.end[2] );

            pose.pose.orientation.w = q.w();
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            msg_path.poses.push_back ( pose );
        }

        robot->pubPaths_.publish ( msg_path );


        msg_route.segments.clear();

        for ( const Checkpoint &cp : route ) {
            tuw_multi_robot_msgs::RouteSegment seg;

            Eigen::Vector2d posStart ( cp.start[0] * mapResolution_, cp.start[1] * mapResolution_ );
            tf::Quaternion qStart;
            qStart.setEuler ( 0, 0, cp.start[2] );

            seg.start.position.x = posStart[0] + mapOrigin_[0];
            seg.start.position.y = posStart[1] + mapOrigin_[1];
            seg.start.orientation.w = qStart.w();
            seg.start.orientation.x = qStart.x();
            seg.start.orientation.y = qStart.y();
            seg.start.orientation.z = qStart.z();

            Eigen::Vector2d posEnd ( cp.end[0] * mapResolution_, cp.end[1] * mapResolution_ );
            tf::Quaternion qEnd;
            qEnd.setEuler ( 0, 0, cp.end[2] );

            seg.end.position.x = posEnd[0] + mapOrigin_[0];
            seg.end.position.y = posEnd[1] + mapOrigin_[1];
            seg.end.orientation.w = qEnd.w();
            seg.end.orientation.x = qEnd.x();
            seg.end.orientation.y = qEnd.y();
            seg.end.orientation.z = qEnd.z();

            seg.segment_id = cp.segId;
            seg.width = segment_width_[cp.segId] * mapResolution_;

            for ( int j = 0; j < cp.preconditions.size(); j++ ) {
                tuw_multi_robot_msgs::RoutePrecondition pc;
                pc.robot_id = active_robots_[cp.preconditions[j].robotId]->robot_name;
                pc.current_route_segment = cp.preconditions[j].stepCondition;
                seg.preconditions.push_back ( pc );
            }

            msg_route.segments.push_back ( seg );
        }

        robot->pubRoute_.publish ( msg_route );
    }

    tuw_multi_robot_msgs::RouterStatus ps;
    ps.id = id_;
    ps.success = 1;
    ps.overall_path_length = ( int32_t ) getOverallPathLength();
    ps.longest_path_length = ( int32_t ) getLongestPathLength();
    ps.priority_scheduling_attemps = ( int32_t ) getPriorityScheduleAttemps();
    ps.speed_scheduling_attemps = ( int32_t ) getSpeedScheduleAttemps();
    ps.duration = ( int32_t ) getDuration_ms();

    pubPlannerStatus_.publish ( ps );
}


float SingleRobotRouterNode::getYaw ( const geometry_msgs::Quaternion &_rot ) {
    double roll, pitch, yaw;

    tf::Quaternion q ( _rot.x, _rot.y, _rot.z, _rot.w );
    tf::Matrix3x3 ( q ).getRPY ( roll, pitch, yaw );
    return yaw;
}


float SingleRobotRouterNode::calcRadius ( const int shape, const std::vector<float> &shape_variables ) const {
    tuw_multi_robot_msgs::RobotInfo ri;
    if ( shape == ri.SHAPE_CIRCLE ) {
        return shape_variables[0];
    }

    return -1;
}


size_t SingleRobotRouterNode::getHash ( const std::vector<signed char> &_map, const Eigen::Vector2d &_origin, const float &_resolution ) {
    std::size_t seed = 0;

    boost::hash_combine ( seed, _origin[0] );
    boost::hash_combine ( seed, _origin[1] );
    boost::hash_combine ( seed, _resolution );

    for ( const signed char &val : _map ) {
        boost::hash_combine ( seed, val );
    }

    return seed;
}

std::size_t SingleRobotRouterNode::getHash ( const std::vector<Segment> &_graph ) {
    std::size_t seed = 0;

    for ( const Segment &seg : _graph ) {
        boost::hash_combine ( seed, seg.width() );
        boost::hash_combine ( seed, seg.length() );
        boost::hash_combine ( seed, seg.getSegmentId() );

        for ( const int &p : seg.getPredecessors() ) {
            boost::hash_combine ( seed, p );
        }

        for ( const int &s : seg.getSuccessors() ) {
            boost::hash_combine ( seed, s );
        }

        for ( const Eigen::Vector2d &vec : seg.getPoints() ) {
            boost::hash_combine ( seed, vec[0] );
            boost::hash_combine ( seed, vec[1] );
        }
    }

    return seed;
}

}
