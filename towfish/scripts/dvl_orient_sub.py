#!/usr/bin/env python

import rospy
from towfish.msg import DVLOrient
import csv
import pandas as pd
import time
import os

class DvlOrientLogger:
    def __init__(self):
        self.dvlvel_sub = rospy.Subscriber('/dvl/orientData', DVLOrient, self.dvlorient_callback)
        self.csv_path = '/home/towfish/Desktop/catkin_ws/src/towfish/csv_files/dvlorient_data.csv'
        if os.path.isfile(self.csv_path):
            pass
        else:
            pd.DataFrame(columns=['time_stamp', 'dvl_time_stamp', 'position_x', 'position_y', 'position_z', 'roll', 'pitch', 'yaw']).to_csv(self.csv_path)

    def dvlorient_callback(self, data):
        data_dict = {"time_stamp": [time.time()],
                     "dvl_time_stamp": [data.header.stamp],
                     "position_x": [data.position.x],
                     "position_y": [data.position.y],
                     "position_z": [data.position.z],
                     "roll": [data.roll],
                     "pitch": [data.pitch],
                     "yaw": [data.yaw]}
        tempdf = pd.DataFrame(data_dict)
        tempdf.to_csv(self.csv_path, mode='a', index=False, header=False)

if __name__ =='__main__':
    try:
        dvlorient_logger = DvlOrientLogger()
        rospy.init_node('dvl_orient_sub', anonymous=True)
      #  print("hello")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

