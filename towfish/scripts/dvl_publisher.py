#!/usr/bin/python3
from cgitb import reset
import socket
import json
import rospy
from time import sleep
from std_srvs.srv import Empty, EmptyResponse
from towfish.msg import DVLVel
from towfish.msg import DVLBeam
from towfish.msg import DVLBeamsArr
from towfish.msg import DVLOrient

def connect():
    global dvl_socket, TCP_IP, TCP_PORT
    dvl_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while(not rospy.is_shutdown()):
        sleep(1)
        try:
            dvl_socket.connect((TCP_IP, TCP_PORT))
            break
        except socket.error as err:
            rospy.logerr("No route to host, DVL might be booting? {}".format(err))
    dvl_socket.settimeout(1)

def reset_dead_reckoning():
    buffer_size = 4096
    cmd_msg = {"command": "reset_dead_reckoning"}
    jsonObj  = json.dumps(cmd_msg) + "\r\n"
    json_encode = jsonObj.encode('utf-8')
    dvl_socket.send(json_encode)
    print("Sent the reset message")
    message = dvl_socket.recv(buffer_size).decode()
    print("Received response")
    data = json.loads(message)

def cbReset(msg):
    reset_dead_reckoning()
    return EmptyResponse()

def calibrate_gyro():
    buffer_size = 4096
    cmd_msg = {"command": "calibrate_gyro"}
    jsonObj  = json.dumps(cmd_msg)
    dvl_socket.send(jsonObj + "\r\n")
    print("Sent the calibrate_gyro message")
    message = dvl_socket.recv(buffer_size).decode()
    print("Received response")
    data = json.loads(message)
    if data["type"]:
        print(data)

def get_config():
    buffer_size = 4096
    cmd_msg = {"command": "get_config"}
    jsonObj  = json.dumps(cmd_msg)
    dvl_socket.send(jsonObj + "\r\n")
    print("Sent the get_config message")
    message = dvl_socket.recv(buffer_size).decode()
    print("Received response")
    data = json.loads(message)
    if data["type"]:
        print(data["success"])

def set_config():
    #Donot call the function, might set wrong values
    buffer_size = 4096
    cmd_msg = {"command": "set_config",
        "parameters":{"speed_of_sound":1481, 
        "mounting_rotation_offset":0.00}}
    jsonObj  = json.dumps(cmd_msg)
    dvl_socket.send(jsonObj + "\r\n")
    print("Sent the set_config message")
    message = dvl_socket.recv(buffer_size).decode()
    print("Received response")
    data = json.loads(message)
    if data["type"]:
        print(data["success"])

def publish_msg(data):
    global pub_vel, pub_orient, pub_beams
    beams = [DVLBeam(), DVLBeam(), DVLBeam(), DVLBeam()]
    if data["type"] == "position_local":
        DVLorient = DVLOrient()
        DVLorient.header.stamp = rospy.Time.now()
        DVLorient.header.frame_id = "dvl_link_1"
        DVLorient.ts = data["ts"]
        DVLorient.position.x = data["x"]
        DVLorient.position.y = data["y"]
        DVLorient.position.z = data["z"]
        DVLorient.roll = data["roll"]
        DVLorient.pitch = data["pitch"]
        DVLorient.yaw = data["yaw"]
        pub_orient.publish(DVLorient)

    elif data["type"] == "velocity":
        DVLvel = DVLVel()
        DVLvel.header.stamp = rospy.Time.now()
        DVLvel.header.frame_id = "dvl_link_2"
        DVLvel.time = data["time"]
        DVLvel.velocity.x = data["vx"]
        DVLvel.velocity.y = data["vy"]
        DVLvel.velocity.z = data["vz"]
        DVLvel.fom = data["fom"]
        DVLvel.altitude = data["altitude"]
        DVLvel.velocity_valid = data["velocity_valid"]
        DVLvel.status = data["status"]
        DVLvel.form = data["format"]
        pub_vel.publish(DVLvel)

        for i in range(4):
            beams[i].id = data["transducers"][i]["id"]
            beams[i].velocity = data["transducers"][i]["velocity"]
            beams[i].distance = data["transducers"][i]["distance"]
            beams[i].rssi = data["transducers"][i]["rssi"]
            beams[i].nsd = data["transducers"][i]["nsd"]
            beams[i].valid = data["transducers"][i]["beam_valid"]

        DVLbeams = DVLBeamsArr()
        DVLbeams.beams = beams
        pub_beams.publish(DVLbeams)

def _process_messages():
    global dvl_socket
    buffer_size = 4096
    message = ""
    while True:
        buffer = dvl_socket.recv(buffer_size).decode()
        if not buffer:
            continue
        message_parts = buffer.split("\r\n")
        message_parts[0] = message + message_parts[0]
        for message_part in message_parts[:-1]:
            try:
                data = json.loads(message_part)
                if data["type"] != "response":
                    publish_msg(data)
            except ValueError:
                print("Could not parse to JSON: " + message_part)
        message = message_parts[-1] if message_parts[-1] else ""

if __name__ == '__main__':
    global dvl_socket, TCP_IP, TCP_PORT, do_log_raw_data, pub_vel, pub_orient, pub_beams

    rospy.init_node('dvl_a50_pub')
    TCP_IP = rospy.get_param("~ip", "192.168.194.95")
    TCP_PORT = rospy.get_param("~port", 16171)
    do_log_raw_data = rospy.get_param("~do_log_raw_data", False)
    connect()
    # set_config()
    pub_vel = rospy.Publisher('dvl/velData', DVLVel, queue_size=10)
    pub_orient = rospy.Publisher('dvl/orientData', DVLOrient, queue_size=10)
    pub_beams = rospy.Publisher('dvl/beamsData', DVLBeamsArr, queue_size=10)
    reset_service = rospy.Service('dvl_a50/reset', Empty, cbReset)
    try:
        _process_messages()
    except rospy.ROSInterruptException:
        dvl_socket.close()
