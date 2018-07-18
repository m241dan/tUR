#include <usb_cam_sim/usb_cam_sim.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

ros::Timer timer;
image_transport::CameraPublisher im_pub;
sensor_msgs::Image img;
sensor_msgs::CameraInfo ci;

void publish_image( const ros::TimerEvent &event )
{
    im_pub.publish( img, ci );
}

int main( int argc, char *argv[] )
{
    std::string img_loc;
    cv::Mat image;
    std_msgs::Header head;
    std::string encoding( "bgr8" );
    cv_bridge::CvImage cv_img;


    ros::init( argc, argv, "cam_sim_node" );
    ros::NodeHandle nh("~");
    nh.param("img_location", img_loc, std::string( "/home/korisd/img.jpg" ) );
    image = cv::imread( img_loc );
    cv_img = cv_bridge::CvImage( head, encoding, image );
    img = *cv_img.toImageMsg();

    image_transport::ImageTransport it( nh );
    im_pub = it.advertiseCamera( "image_raw", 1 );
    timer = nh.createTimer( ros::Duration(1), publish_image );

    ros::spin();

    return 0;
}
