/*
 * Copyright (c) 2017, <copyright holder> <email>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#define POT_HIGH 1.0e10

#include <tuw_global_router/router_node.h>
#include <tuw_global_router/srr_utils.h>
#include <tuw_multi_robot_msgs/Route.h>
#include <chrono>
#include <boost/functional/hash.hpp>
#include <boost/regex.hpp>
#include <tf/tf.h>
#include <unordered_map>
#include <libssh/libssh.h>

//TODO add Weights from robots...

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "tuw_multi_robot_router" ); /// initializes the ros node with default name
    ros::NodeHandle n;

    ros::Rate r ( 1 );

    multi_robot_router::Router_Node node ( n );

    while ( ros::ok() ) {
        r.sleep();
        ros::spinOnce();
        node.monitorExecution();
        node.updateTimeout ( r.expectedCycleTime().toSec() );
   }

    return 0;
}


namespace multi_robot_router {

Router_Node::Router_Node ( ros::NodeHandle &_n ) : Router(),
    n_ ( _n ),
    n_param_ ( "~" ),
    monitor_enabled_ ( false ),
    attempts_total_(0),
    attempts_successful_(0),
    sum_processing_time_total_(.0),
    sum_processing_time_successful_(.0){
    id_ = 0;

    n_param_.param<bool> ( "single_robot_mode", single_robot_mode_, false );

    n_param_.param<std::string> ( "frame_id", frame_id_, frame_id_ );

    // Static subscriptions
    subMap_ = n_.subscribe ( "map", 1, &Router_Node::mapCallback, this );
    subVoronoiGraph_ = n_.subscribe ( "segments", 1, &Router_Node::graphCallback, this );
    subRobotInfo_ = n_.subscribe ( "robot_info" , 10000, &Router_Node::robotInfoCallback, this );
    subSingleRobotIdGoal_ = n_.subscribe ( "labelled_goal", 1, &Router_Node::labelledGoalCallback, this );

    if ( single_robot_mode_) {
        // Single Robot Mode
        ROS_INFO("Single Robot Mode");
        subSingleRobotGoal_ = n_.subscribe ( "goal", 1, &Router_Node::goalCallback, this );
    } else {
        // Multi Robot Mode
        ROS_INFO("Multi Robot Mode");
        subGoalSet_ = n_.subscribe ( "goals" , 1, &Router_Node::goalsCallback, this );
    }

    // Static publishers
    pubPlannerStatus_ = n_.advertise<tuw_multi_robot_msgs::RouterStatus> ( "planner_status", 1 );

    // Dynamic reconfigure
    call_type = boost::bind ( &Router_Node::parametersCallback, this, _1, _2 );
    param_server.setCallback ( call_type );

    // Registration/unregistration services - J. Mendes
    n_param_.param<int>("max_robots", max_robots_, 100);
    n_param_.param<double>("noinfo_timeout", noinfo_timeout_, 15.);
    num_of_robots_ = 0;
    register_service_ = n_.advertiseService("register_robot", &Router_Node::registerNewRobotCB, this);
    // unregister_service_ = nh.advertiseService("unregister_robot", &Router_Node::unregisterRobotCB, this);
    ROS_INFO("Register robot service advertised");
}

bool ssh_authentication(std::string ssh_server)
{
    // -------------------------------------------------------------------------
    // SSH authentification - authorized_keys have to be up to date
    // -------------------------------------------------------------------------
    int rc;
    ssh_session session = ssh_new();
    if (session == NULL)
    {
        ROS_ERROR(
            "Cannot create ssh sesssion");
        return false;
    }

    // get router url/ip from a service
    // router_hn = "tank.mesh.tpg.argentan.ifollow";
    ssh_options_set(session, SSH_OPTIONS_HOST, ssh_server.c_str());

    rc = ssh_connect(session);
    if (rc != SSH_OK)
    {
        ROS_ERROR("Error connecting.");
        return false;
    }

    rc = ssh_userauth_publickey_auto(session, NULL, NULL);

    switch(rc)
    {
        case SSH_AUTH_ERROR:
            ROS_INFO("A serious error happened");
            break;
        case SSH_AUTH_DENIED:
            ROS_INFO("The server doesn't accept that public key as an authentication token. Try another key or another method");
            break;
        case SSH_AUTH_PARTIAL:
            ROS_INFO("You've been partially authenticated, you still have to use another method");
            break;
        case SSH_AUTH_SUCCESS:
            ROS_INFO("The public key is accepted!");
            break;
        case SSH_AUTH_AGAIN:
            ROS_INFO("In nonblocking mode, you've got to call this again later");
    }
    if (rc != SSH_AUTH_SUCCESS)
    {
        ROS_ERROR(
            "Cannot register the robot. Router authentication failed. %s", ssh_get_error(session));
        // Kill the node with error message
        // exit();
        // return false;
        return false;
    }
    // auth ok, close the session and proceed
    ssh_disconnect(session);
    ssh_free(session);
    // -------------------------------------------------------------------------
    return true;
}

bool Router_Node::registerNewRobotCB(tuw_multi_robot_msgs::RegisterRobot::Request& req,
                                     tuw_multi_robot_msgs::RegisterRobot::Response& res)
{
    std::lock_guard<std::mutex> lock(reg_mutex_);
    if (num_of_robots_ > max_robots_ - 1)
    {
        ROS_ERROR(
            "Cannot register new robot. Maximum number of robots that can be registered is %d", max_robots_);
        // res.ack = false;
        return false;
    }

    std::string robot_id = req.id;
    std::replace(robot_id.begin(), robot_id.end(), '_', '-');

    if (!ssh_authentication(robot_id)) {
      ROS_ERROR("Registration failed");
      return false;
    }

    // std::ostringstream ss;
    // ss << std::setw(5) << std::setfill('0') << 12 << "\n";
    // std::string s2(ss.str());

    // res.id = "robot_" + std::to_string(num_of_robots_);
    // res.id = req.ros_namespace.substr(1, req.ros_namespace.length());         // FIXME

    // PublisherMap::iterator robot_it = robots_.find(res.id);
    // if (robot_it == robots_.end())
    // {
    // pubs_[res.id] = std::make_shared<ros::Publisher>(nh_.advertise<mrs_lp_msgs::CommList>(req.ros_namespace + "/comm_list", 1));
    // }

    ROS_INFO_STREAM("Registered new robot: " << req.id);
    // ROS_INFO_STREAM("Registered new robot: Robot_" << res.id << " at host " << req.robot_hostname);
    num_of_robots_++;
    ids_.push_back(req.id);

    // loc_subs_[res.id] = std::make_shared<ros::Subscriber>(nh_.subscribe("/" + res.id + "/localization", 1, &ResgistrationCenter::locCB_, this));

    // locCB_last_call_[res.id] = ros::Time::now();

    char hostname[HOST_NAME_MAX];
    gethostname(hostname, HOST_NAME_MAX);
    res.id = std::string(hostname);

    return true;
}


void Router_Node::monitorExecution() {

    for ( const RobotInfoPtr robot: active_robots_ ) {
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


    ROS_INFO("Number of active robots : %d", goals_msg_.robots.size());

}


void Router_Node::updateTimeout ( const float _secs ) {
    //Todo update timeouts and clear old messages
    for ( auto robot : subscribed_robots_ ) {
        robot->updateOnlineStatus ( _secs );
    }
}


void Router_Node::goalCallback ( const geometry_msgs::PoseStamped &msg ) {

    if ( subscribed_robots_.size() != 1 ) {
        ROS_WARN ( "No robot subscribed, a tuw_multi_robot_msgs::RobotInfo should be published in order to know where the robot is located");
        ROS_WARN ( "Use a local behavior controller to publish regual a RobotInfo msg");
        return;
    }
    
    tuw_multi_robot_msgs::RobotGoalsArray goalsArray;
    goalsArray.header = msg.header;
    goalsArray.robots.resize(1);
    tuw_multi_robot_msgs::RobotGoals &goals = goalsArray.robots[0];
    goals.robot_name = subscribed_robots_[0]->robot_name;
    goals.destinations.resize(1);
    goals.destinations[0] = msg.pose;
    goalsCallback ( goalsArray );
}


void Router_Node::labelledGoalCallback ( const tuw_multi_robot_msgs::RobotGoals &_goal ) {

    ROS_DEBUG("Labelled goal received");

    // Check if the robot associated with goal is subscribed to the router
    bool is_robot_subscribed=false;
    for(auto it=subscribed_robots_.begin();it!=subscribed_robots_.end();it++) {
        if(!_goal.robot_name.compare((*it)->robot_name)) {
            is_robot_subscribed=true;
            break;
        }
    }

    // If the robot is subscribed
    if(is_robot_subscribed) {
      // If goal already exists for the robot, replace it
      bool found_goal=false;
      for(uint32_t i=0; i<goals_msg_.robots.size(); i++) {
        if(!goals_msg_.robots.at(i).robot_name.compare(_goal.robot_name)) {
          goals_msg_.robots.at(i)=_goal;
          found_goal=true;
          ROS_DEBUG("multi robot router : Found new goal for robot");
          break;
        }
          
      }

      // If no existing goal for this robot, then add it
      if(!found_goal) {
        goals_msg_.robots.push_back(_goal);
        ROS_DEBUG("multi robot router : no goal found for this robot, adding to the planner");
      }

      // Plan the route
      plan();
      
    } else {
      ROS_ERROR("Multi Robot Router: the robot associated with the goal has not subscribed to the router");
    }

}


void Router_Node::goalsCallback ( const tuw_multi_robot_msgs::RobotGoalsArray &_goals ) {

    ROS_DEBUG("Group of goals received");
    ROS_INFO ( "%s: Number of active robots %lu", n_param_.getNamespace().c_str(), active_robots_.size() );
    // Update goals
    goals_msg_ = _goals;
     // Plan the route
    plan();
 
}


void Router_Node::parametersCallback ( tuw_multi_robot_router::routerConfig &config, uint32_t level ) {
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


void Router_Node::mapCallback ( const nav_msgs::OccupancyGrid &_map ) {
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


void Router_Node::robotInfoCallback ( const tuw_multi_robot_msgs::RobotInfo &_robotInfo ) {

    auto robot = RobotInfo::findObj ( subscribed_robots_, _robotInfo.robot_name );
    // If new robot connected, add it to the router
    if ( robot == subscribed_robots_.end() ) {
        // Create new entry
        RobotInfoPtr robot_new = std::make_shared<RobotInfo> ( _robotInfo );
        robot_new->initTopics ( n_ , !single_robot_mode_);
        subscribed_robots_.push_back ( robot_new );
    } else {
        ( *robot )->updateInfo ( _robotInfo );
    }

    robot_radius_max_ = 0;
    for ( RobotInfoPtr &r: subscribed_robots_ ) {
        if ( r->radius() > robot_radius_max_ )
            robot_radius_max_ = r->radius();
    }

    if(single_robot_mode_ && (subscribed_robots_.size() > 1)){
        ROS_WARN("More than one robot subscribed, but the MRRP is in single robot mode");
    }
}


void Router_Node::graphCallback ( const tuw_multi_robot_msgs::Graph &msg ) {
    std::vector<Segment> graph;

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
            graph.emplace_back ( segment.id, points, successors, predecessors,  segment.width); //segment.width);
        } else {
            graph.emplace_back ( segment.id, points, successors, predecessors, 0 );
        }
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


void Router_Node::plan() {

    ROS_DEBUG("Start planning");

    planner_prepared_ = false;
    planner_prepared_ = preparePlanning ( radius_, starts_, goals_, goals_msg_, robot_names_ );

    if ( planner_prepared_ && got_map_ && got_graph_ ) {
        ROS_DEBUG("Multi-robot router : planning preparation successful");
        attempts_total_++;
        // Try to find a plan
        auto t1 = std::chrono::high_resolution_clock::now();
        plan_found_=false;
        plan_found_ = makePlan ( starts_, goals_, radius_, distMap_, mapResolution_, mapOrigin_, graph_, robot_names_ );
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


bool Router_Node::preparePlanning ( std::vector<float> &_radius, std::vector<Eigen::Vector3d> &_starts, std::vector<Eigen::Vector3d> &_goals, const tuw_multi_robot_msgs::RobotGoalsArray &goal_msg, std::vector<std::string> &robot_names ) {
    bool retval = true;
    active_robots_.clear();
    _starts.clear();
    _goals.clear();
    _radius.clear();

    // Store the name of the robots with a wanted goal
    std::vector<std::string> active_robots_name;
    for ( int k = 0; k < goal_msg.robots.size(); k++ ) {
      active_robots_name.push_back(goal_msg.robots[k].robot_name);
    }

    for ( int k = 0; k < subscribed_robots_.size(); k++ ) {
        RobotInfoPtrIterator active_robot = RobotInfo::findObj ( subscribed_robots_, subscribed_robots_[k]->robot_name );
        auto active_goal=goal_msg.robots.end();
        // For each robot, if there is a new goal for it, then use it
        for(auto it=goal_msg.robots.begin();it!=goal_msg.robots.end();it++) {
            if(!subscribed_robots_[k]->robot_name.compare(it->robot_name) ) {
                active_goal=it;
            }
        }


        if ( active_goal != goal_msg.robots.end() ) {
            const tuw_multi_robot_msgs::RobotGoals &route_request = *active_goal;
            active_robots_.push_back ( *active_robot );

            _radius.push_back ( ( *active_robot )->radius() );
            if ( route_request.destinations.size() > 1 ) {
                geometry_msgs::Pose p = route_request.destinations[0];
                _starts.push_back ( Eigen::Vector3d ( p.position.x, p.position.y, getYaw ( p.orientation ) ) );
            } else {
                _starts.push_back ( ( *active_robot )->getPose() );
            }

            geometry_msgs::Pose p = route_request.destinations.back();
            _goals.push_back ( Eigen::Vector3d ( p.position.x, p.position.y, getYaw ( p.orientation ) ) );
        } else {
            // If the robot has no new goal, then just assign same start and goal position
            _radius.push_back ( ( *active_robot )->radius() );
            _starts.push_back ( ( *active_robot )->getPose() );
            _goals.push_back ( ( *active_robot )->getPose() );
            continue;

        }

    }


    robot_names.resize ( subscribed_robots_.size() );
    for ( size_t i=0; i < subscribed_robots_.size(); i++ ) {
        robot_names[i] = subscribed_robots_[i]->robot_name;
    }

    return retval;
}


void Router_Node::publishEmpty() {
    if(publish_routing_table_ == false) return;
    ROS_INFO ( "%s: No Plan found!!!!, publishing empty plan", n_param_.getNamespace().c_str());
    time_first_robot_started_ = ros::Time::now();
    finished_robots_.clear();
    nav_msgs::Path msg_path;
    tuw_multi_robot_msgs::Route msg_route;
    msg_path.header.seq = 0;
    msg_path.header.stamp = time_first_robot_started_;
    msg_path.header.frame_id = frame_id_;
    msg_route.header = msg_path.header;

    for ( RobotInfoPtr &robot: subscribed_robots_ ) {
        robot->pubPaths_.publish ( msg_path );
        robot->pubRoute_.publish ( msg_route );
    }

    mrrp_status_.id = id_;
    mrrp_status_.success = 0;
    mrrp_status_.duration = getDuration_ms();
    pubPlannerStatus_.publish ( mrrp_status_ );
}


void Router_Node::publish() {
    if(publish_routing_table_ == false) return;
    ROS_INFO ( "%s: Plan found :-), publishing plan", n_param_.getNamespace().c_str());
    time_first_robot_started_ = ros::Time::now();
    finished_robots_.clear();
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
            seg.width = graph_[cp.segId].width() * mapResolution_;


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


float Router_Node::getYaw ( const geometry_msgs::Quaternion &_rot ) {
    double roll, pitch, yaw;

    tf::Quaternion q ( _rot.x, _rot.y, _rot.z, _rot.w );
    tf::Matrix3x3 ( q ).getRPY ( roll, pitch, yaw );
    return yaw;
}


float Router_Node::calcRadius ( const int shape, const std::vector<float> &shape_variables ) const {
    tuw_multi_robot_msgs::RobotInfo ri;
    if ( shape == ri.SHAPE_CIRCLE ) {
        return shape_variables[0];
    }

    return -1;
}


size_t Router_Node::getHash ( const std::vector<signed char> &_map, const Eigen::Vector2d &_origin, const float &_resolution ) {
    std::size_t seed = 0;

    boost::hash_combine ( seed, _origin[0] );
    boost::hash_combine ( seed, _origin[1] );
    boost::hash_combine ( seed, _resolution );

    for ( const signed char &val : _map ) {
        boost::hash_combine ( seed, val );
    }

    return seed;
}

std::size_t Router_Node::getHash ( const std::vector<Segment> &_graph ) {
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


} // namespace multi_robot_router
