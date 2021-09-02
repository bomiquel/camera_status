#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import message_filters
import numpy as np
import pandas as pd
from std_msgs.msg import Float32
from sensor_msgs.msg import CameraInfo


TIME_TO_CONVERGENCE = 10


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
        self.df = pd.DataFrame(columns =['TimeStamp'])
        self.cont_non_convergence = 1
        self.last_callback = 'timer'

        
    def image_callback(self, l_info_msg, r_info_msg):

        dict = {'TimeStamp':[l_info_msg.header.stamp.secs]}
        aux_df = pd.DataFrame(dict)
        self.df = pd.concat([self.df, aux_df], ignore_index = True)

        print(self.df)

        if (self.df['TimeStamp'].iloc[-1] - self.df['TimeStamp'].iloc[0]) >= TIME_TO_CONVERGENCE:

            self.df = self.df.iloc[1: , :]

        self.last_callback = 'image'


    def timer_callback(self, publication_timer):

        if self.last_callback is 'image':

            if self.cont_non_convergence <= TIME_TO_CONVERGENCE:

                frame_rate = (float)(len(self.df) / self.cont_non_convergence)

                self.cont_non_convergence += 1
            
            else:

                frame_rate = (len(self.df) / TIME_TO_CONVERGENCE)

        else:

            if len(self.df) != 0:
                
                self.df = self.df.drop(self.df.index[range(len(self.df))])
                
            self.cont_non_convergence = 1

            frame_rate = len(self.df)

        self.camera_status_pub.publish(frame_rate)

        self.last_callback = 'timer'


if __name__ == '__main__':
    rospy.init_node('camera_status')
    camera_status = CameraStatus()
    rospy.spin()

