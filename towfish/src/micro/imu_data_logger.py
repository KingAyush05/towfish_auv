#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import csv
import pandas as pd
import time
import os

class ImuDataLogger:
    def __init__(self):
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.csv_path = '/home/towfish/Desktop/catkin_ws/src/towfish/csv_files/imu_data.csv'
        if os.path.isfile(self.csv_path):
            pass
        else:
            pd.DataFrame(columns=["time_stamp", "sequence_number", "imu_secs", "imu_nsecs", "frame_id", "quat_x", "quat_y", "quat_z", "omega_x", "omega_y", "omega_z", "acc_x", "acc_y", "acc_z"]).to_csv(self.csv_path)

    def imu_callback(self, data):
        if not data.header.seq % 10 == 0:
            return
        t0 = time.time()
        data_dict = {"time_stamp": [time.time()],
                     "sequence_number": [data.header.seq],
                     "imu_secs": [data.header.stamp.secs],
                     "imu_nsecs": [data.header.stamp.nsecs],
                     "frame_id": [data.header.frame_id],
                     "quat_x": [data.orientation.x],
                     "quat_y": [data.orientation.y],
                     "quat_z": [data.orientation.z],
                     "quat_w": [data.orientation.w],
                     "omega_x": [data.angular_velocity.x],
                     "omega_y": [data.angular_velocity.y],
                     "omega_z": [data.angular_velocity.z],
                     "acc_x": [data.linear_acceleration.x],
                     "acc_y": [data.linear_acceleration.y],
                     "acc_z": [data.linear_acceleration.z]}
        tempdf = pd.DataFrame(data_dict)
        t1 = time.time()
        #print("callback done")
        t2 = time.time()
        tempdf.to_csv(self.csv_path, mode='a', index=False, header=False)
        t3 = time.time()

        #print(f"Time to make DataFrame : {t1 - t0}")
        #print(f"Time to perform callback : {t2 - t1}")
        #print(f"Time to save DataFrame : {t3 - t2}")

if __name__ == '__main__':
    try:
        imu_logger = ImuDataLogger()
        rospy.init_node('imu_data_logger', anonymous=True)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

