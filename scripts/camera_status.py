#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import message_filters
import numpy as np
import pandas as pd
from std_msgs.msg import Float32
from sensor_msgs.msg import CameraInfo


class CameraStatus:

    def __init__(self):

        # Topic subscribers
        left_camera_info_sub = message_filters.Subscriber("/stereo_down/left/camera_info", CameraInfo)
        right_camera_info_sub = message_filters.Subscriber("/stereo_down/right/camera_info", CameraInfo)

        # Topic synchronization
        ts =  message_filters.ApproximateTimeSynchronizer([left_camera_info_sub, right_camera_info_sub], 1, 1)

        # Internal callbacks
        ts.registerCallback(self.image_callback)
        self.publication_timer = rospy.Timer(rospy.Duration(1), self.timer_callback)
        
        # Publisher
        self.camera_status_pub = rospy.Publisher('/stereo_down/camera_status', Float32, queue_size = 1)

        # Variables
        self.df = pd.DataFrame(columns = ['TimeStamp'])
        self.cont_non_convergence = 1

        # Params
        self.TIME_TO_CONVERGENCE = rospy.get_param('~time_to_convergence', default = 5)
        self.COURTESY_TIME = rospy.get_param('~courtest_time', default = 3)
        

    # Function that introduces in a dataframe the time stamp of each image received by the node.
    # The time stamps will be stored in the dataframe for TIME_TO_CONVERGENCE seconds.
    def image_callback(self, l_info_msg, r_info_msg):

        dict = {'TimeStamp':[l_info_msg.header.stamp.secs]}
        aux_df = pd.DataFrame(dict)
        self.df = pd.concat([self.df, aux_df], ignore_index = True)

        if (self.df['TimeStamp'].iloc[-1] - self.df['TimeStamp'].iloc[0]) >= self.TIME_TO_CONVERGENCE:

            self.df = self.df.iloc[1: , :]


    # Function that publishes every second the frame rate of the images. The frame rate is 
    # calculated using the dataframe length divided by the convergence time. While the 
    # dataframe is being filled, the length of the dataframe divided by the number of seconds
    # the node has been running is used to calculate the frame rate. If it is detected that 
    # the time elapsed between the last image and the current time is greater than or equal 
    # to the courtesy time, the dataframe is restarted.
    def timer_callback(self, publication_timer):

        current_time = int(rospy.get_rostime().to_sec())

        if len(self.df) == 0:

            rospy.logwarn('No received images.')

            frame_rate = len(self.df)

        else:

            if (current_time - self.df['TimeStamp'].iloc[-1]) < self.COURTESY_TIME:

                if self.cont_non_convergence <= self.TIME_TO_CONVERGENCE:

                    frame_rate = (float)(len(self.df) / self.cont_non_convergence)

                    self.cont_non_convergence += 1
                
                else:

                    frame_rate = (len(self.df) / self.TIME_TO_CONVERGENCE)

            else:

                rospy.logwarn('No received images.')

                self.df = self.df.drop(self.df.index[range(len(self.df))])
                    
                self.cont_non_convergence = 1

                frame_rate = len(self.df)

        self.camera_status_pub.publish(frame_rate)


if __name__ == '__main__':
    rospy.init_node('camera_status')
    camera_status = CameraStatus()
    rospy.spin()

