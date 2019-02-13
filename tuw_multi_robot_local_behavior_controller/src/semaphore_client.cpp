#include <stoplights_client/semaphore_client.h>

namespace stoplights_client {

SemaphoreClient::SemaphoreClient(const ros::NodeHandle & _n, const std::string & _robot_id, const double & _robot_width) : 
    n_(_n),
    robot_id_(_robot_id),
    robot_width_(_robot_width)
{

    // Creates client for the server services
    lights_client_in_ = n_.serviceClient<ifollow_nav_msgs::StoplightsSys>("stoplights_sys_in");
    lights_client_out_ = n_.serviceClient<ifollow_nav_msgs::StoplightsSys>("stoplights_sys_out");

}

bool SemaphoreClient::requestAuthorization(const uint32_t & actual_segment_id, const uint32_t & restricted_segment_id) {

    ifollow_nav_msgs::StoplightsSys srv;
    srv.request.robot_id = robot_id_;
    srv.request.access_seg_id = restricted_segment_id;
    srv.request.curr_seg_id = actual_segment_id;
    srv.request.robot_width = robot_width_;

    if (!lights_client_in_.waitForExistence(ros::Duration(1)))
    {

        ROS_ERROR_STREAM("Timeout, service not available");
        return false;

    } else {
         
        bool srv_call_success = lights_client_in_.call(srv);
        if (srv_call_success)
            return srv.response.ack;
        else
            return false; 
        
    }
    
}

bool SemaphoreClient::advertiseLeavingSegment(const uint32_t & leaving_segment_id, const uint32_t & entering_segment_id) {

    ifollow_nav_msgs::StoplightsSys srv;
    srv.request.robot_id = robot_id_;
    srv.request.access_seg_id = entering_segment_id;
    srv.request.curr_seg_id = leaving_segment_id;
    srv.request.robot_width = robot_width_;

    if (!lights_client_out_.waitForExistence(ros::Duration(1)))
    {

        ROS_ERROR_STREAM("Timeout, service not available");
        return false;

    } else {
         
        bool srv_call_success = lights_client_out_.call(srv);
        if (srv_call_success)
            return true;
        else
            return false; 
        
    }

}


} // namespace stoplights_client
