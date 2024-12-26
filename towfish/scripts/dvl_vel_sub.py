#!/usr/bin/env python

import rospy
from towfish.msg import DVLVel
import csv
import pandas as pd
import time
import os

class DvlVelLogger:
    def __init__(self):
        self.dvlvel_sub = rospy.Subscriber('/dvl/velData', DVLVel, self.dvlvel_callback)
        self.csv_path = '/home/towfish/Desktop/catkin_ws/src/towfish/csv_files/dvlvel_data.csv'
        if os.path.isfile(self.csv_path):
            pass
        else:
            pd.DataFrame(columns=['time_stamp', 'dvl_time_stamp', 'velocity_x', 'velocity_y', 'velocity_z']).to_csv(self.csv_path)

    def dvlvel_callback(self, data):
        t1 = time.time() 
        data_dict = {"time_stamp": [time.time()],
                     "dvl_time_stamp": [data.header.stamp], 
                     "velocity_x": [data.velocity.x],
                     "velocity_y": [data.velocity.y],
                     "velocity_z": [data.velocity.z]}
        tempdf = pd.DataFrame(data_dict)
        tempdf.to_csv(self.csv_path, mode='a', index=False, header=False)

if __name__ =='__main__':
    try:
        dvlvel_logger = DvlVelLogger()
        rospy.init_node('dvl_vel_sub', anonymous=True)
      #  print("hello")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

