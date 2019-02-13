#ifndef SEMAPHORE_CLIENT_H
#define SEMAPHORE_CLIENT_H

#include <ros/ros.h>
#include <ifollow_nav_msgs/StoplightsSys.h>

namespace stoplights_client
{
    class SemaphoreClient {
        public:
            SemaphoreClient(const ros::NodeHandle & _n, const std::string & _robot_id, const double & _robot_width);
            ~SemaphoreClient() { };
            bool requestAuthorization(const uint32_t & actual_segment_id, const uint32_t & restricted_segment_id);
            bool advertiseLeavingSegment(const uint32_t & leaving_segment_id, const uint32_t & entering_segment_id);
        private:
            ros::NodeHandle n_;
            ros::ServiceClient lights_client_in_;
            ros::ServiceClient lights_client_out_;
            std::string robot_id_;
            double robot_width_;
    };

}  // namespace stoplights_client

#endif  // SEMAPHORE_CLIENT_H
