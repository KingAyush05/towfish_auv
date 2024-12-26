#!/usr/bin/python3
import ms5837
import time
import numpy as np
import os
import pandas as pd
import rospy
from towfish.msg import PsData

def ps_init():
    global sensor
    # We must initialize the sensor before reading it 
    while True:
        try:
            while not sensor.init():
                print("Pressure sensor found! Init failed.")
            break
        except OSError:
            print("Can't find the pressure sensor. Check for loose connections. Try running 'i2cdetect -y 1'.")
        time.sleep(1) 

def publish():
    global sensor, pub_pressure, csv_path
    
    rate = rospy.Rate(10)
    data_dict = {}
    while not rospy.is_shutdown():
        try: 
            if sensor.read():
                ps_msg = PsData()
                ps_msg.time = time.time()
                ps_msg.pressure = sensor.pressure()
                ps_msg.temperature = sensor.temperature()
                pub_pressure.publish(ps_msg)
                data_dict.update({'time': [ps_msg.time],
                                  'pressure': [ps_msg.pressure],
                                  'ps_temp': [ps_msg.temperature]})
                pd.DataFrame.from_dict(data_dict).to_csv(csv_path, mode='a', index=False, header=False)
            else:   
                print("Sensor read failed!")
        except Exception as e: 
            print("Pressure sensor disconnected!\n", e)

        rate.sleep()

if __name__ == '__main__':
    global sensor, pub_pressure, csv_path

    sensor = ms5837.MS5837_30BA() # Default I2C bus is 1 (Raspberry Pi 3)
    ps_init()
    csv_path = '/home/towfish/Desktop/catkin_ws/src/towfish/csv_files/ps_data.csv'
    if os.path.isfile(csv_path):
        pass
    else:
        pd.DataFrame(columns=['time', 'pressure', 'ps_temperature']).to_csv(csv_path)
    rospy.init_node('ps_pub')
    pub_pressure = rospy.Publisher('/ps/data', PsData, queue_size=10)
    publish()
