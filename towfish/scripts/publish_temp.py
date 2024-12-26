#!/usr/bin/env python3
import time
import subprocess
import pandas as pd
import os
import rospy
from towfish.msg import Temperature
try: 
    from scd4x import SCD4X
except:
    print("Try pip3 install scd4x")
    exit(1)


def scd4x_init():
    global sensor, sensor_flag

    try:
        sensor = SCD4X(quiet=False)
        sensor_flag = True
    except Exception as e:
        print("SCD40 (Temperature Sensor): ", e)
        print("Sensor not detected. Check for loose connections.")

    if sensor_flag:
        sensor.start_periodic_measurement()

def read_ambient_temperature():
    global sensor

    res = sensor.measure()
    ambient_temp = res[1]
    return ambient_temp

def read_cpu_temperature(): 
    result = subprocess.run(["cat", "/sys/class/thermal/thermal_zone0/temp"], capture_output=True)
    b = [byte-48 for byte in result.stdout]
    b.pop(-1)
    cpu_temp = int(''.join(map(str, b)))/1000
    return cpu_temp

def publish():
    global sensor, pub_temperature, csv_path
    
    rate = rospy.Rate(10)
    data_dict = {}
    while not rospy.is_shutdown():
        cpu_temp = read_cpu_temperature()
        temp_msg = Temperature()
        temp_msg.time = time.time()
        temp_msg.cpu_temp = cpu_temp
        data_dict.update({'time': [temp_msg.time],
                          'cpu_temp': [temp_msg.cpu_temp]})
        if sensor_flag:
            ambient_temp = read_ambient_temp()
            temp_msg.ambient_temp = ambient_temp
            data_dict.update({'ambient_temp': [temp_msg.ambient_temp]})
        pd.DataFrame.from_dict(data_dict).to_csv(csv_path, mode='a', index=False, header=False)
        pub_temperature.publish(temp_msg)
        rate.sleep()

if __name__ == '__main__': 
    global sensor, sensor_flag, pub_temperature, csv_path

    sensor_flag = False
    scd4x_init()
    csv_path = '/home/towfish/Desktop/catkin_ws/src/towfish/csv_files/temp_data.csv'
    if os.path.isfile(csv_path):
        pass
    else:
        pd.DataFrame(columns=['time', 'cpu_temp', 'ambient_temp']).to_csv(csv_path)
    rospy.init_node('temp_pub')
    pub_temperature = rospy.Publisher('/temp/data', Temperature, queue_size=10)
    publish()
