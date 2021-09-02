/* Includes. */
#include <iostream>
#include <ros/ros.h>
// #include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

ros::Timer timer_ ;

ros::Publisher camera_status_pub_ ;

// std_msgs::Bool camera_status_msg ;

std_msgs::Float32 camera_status_msg ;

void callback(const sensor_msgs::CameraInfoConstPtr& l_info_msg, const sensor_msgs::CameraInfoConstPtr& r_info_msg ){

    timer_.stop() ;
    timer_.start() ;

    camera_status_msg.data = 1 ;

    camera_status_pub_.publish(camera_status_msg) ;
}

void timerCallback(const ros::TimerEvent&){

    camera_status_msg.data = 0 ;

    camera_status_pub_.publish(camera_status_msg) ;

}

int main(int argc, char **argv){

    ros::init(argc, argv, "camera_status") ;

    ros::NodeHandle nh ;

    message_filters::Subscriber<sensor_msgs::CameraInfo> left_camera_info_sub(nh, "/stereo_down/left/camera_info", 1) ;
    message_filters::Subscriber<sensor_msgs::CameraInfo> right_camera_info_sub(nh, "/stereo_down/right/camera_info", 1) ;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> sync_pol1 ;
    message_filters::Synchronizer<sync_pol1> sync1(sync_pol1(70), left_camera_info_sub, right_camera_info_sub) ;

    sync1.registerCallback(boost::bind(&callback, _1, _2)) ;

    timer_ = nh.createTimer(ros::Duration(1), timerCallback);

    camera_status_pub_ = nh.advertise<std_msgs::Float32>("/stereo_down/camera_status", 1) ;

    ros::spin() ;

    ros::shutdown() ;

    return 0 ;

}