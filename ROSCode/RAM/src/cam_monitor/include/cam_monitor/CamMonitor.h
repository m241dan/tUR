//
// Created by korisd on 7/7/18.
//

#ifndef CAMMONITOR_H
#define CAMMONITOR_H

#include <cam_monitor/monitor.h>

class CamMonitor
{
    public:
        CamMonitor();
    protected:
        ros::NodeHandle         _node_handle;
        ros::Subscriber         _image_sub;
        ros::Subscriber         _time_sub;
        ros::ServiceServer      _take_snap;
        rosgraph_msgs::Clock    _clock;

        cv_bridge::CvImagePtr   _recent_img;
        std::string             _img_topic;
        std::string             _video_location;
        std::string             _snap_location;
        int                     _throttle;
        int                     _throttle_count;

        void timeCallback( const rosgraph_msgs::Clock::ConstPtr &msg );
        void imageCallback( const sensor_msgs::Image::ConstPtr &msg );
        bool serviceCallback( cam_monitor::SnapRequest &request, cam_monitor::SnapResponse &response );

};


#endif
