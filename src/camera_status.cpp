/* Includes. */
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

ros::Timer timer_1_ ;

ros::Timer timer_10_ ;

ros::Publisher camera_status_pub_ ;

int cont_image_ = 0 ;

int aux_ = 0 ;

std_msgs::Float32 frame_rate_msg_ ;

int publication_frecuency_ = 1 ; 

int publication_average_ = 10 ;

bool average = false ;


void callback(const sensor_msgs::CameraInfoConstPtr& l_info_msg, const sensor_msgs::CameraInfoConstPtr& r_info_msg ){

    cont_image_++ ;

}


void timer_publication(const ros::TimerEvent&){

    if (!average){

        frame_rate_msg_.data = (cont_image_ - aux_) ;

        aux_ = cont_image_ ;

        ROS_INFO("Hello there") ;

    }

    camera_status_pub_.publish(frame_rate_msg_) ;

}


void timer_average(const ros::TimerEvent&){

    frame_rate_msg_.data = (cont_image_ / publication_average_) ;

    cont_image_ = 0 ;

    average = true ;

}



int main(int argc, char **argv){

    ros::init(argc, argv, "camera_status") ;

    ros::NodeHandle nh ;

    message_filters::Subscriber<sensor_msgs::CameraInfo> left_camera_info_sub(nh, "/stereo_down/left/camera_info", 1) ;
    message_filters::Subscriber<sensor_msgs::CameraInfo> right_camera_info_sub(nh, "/stereo_down/right/camera_info", 1) ;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> sync_pol1 ;
    message_filters::Synchronizer<sync_pol1> sync1(sync_pol1(70), left_camera_info_sub, right_camera_info_sub) ;

    sync1.registerCallback(boost::bind(&callback, _1, _2)) ;

    timer_1_ = nh.createTimer(ros::Duration(publication_frecuency_), timer_publication) ;

    timer_10_ = nh.createTimer(ros::Duration(publication_average_), timer_average);

    camera_status_pub_ = nh.advertise<std_msgs::Float32>("/stereo_down/camera_status", 1) ;

    ros::spin() ;

    ros::shutdown() ;

    return 0 ;

}