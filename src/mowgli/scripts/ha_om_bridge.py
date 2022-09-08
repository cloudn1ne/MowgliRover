#!/usr/bin/python3
# Mowgli/OpenMower HA to OM Bridge V 1.1
# (c) Georg Swoboda <cn@warp.at> 2022
#
# https://github.com/cloudn1ne/Mowgli
#
#
# v1.0: inital release
# v1.1: added config file support
#
#
import random
import time
import sys
import signal
import rospy
import configparser
from paho.mqtt import client as mqtt_client
from mower_msgs.srv import HighLevelControlSrv,HighLevelControlSrvRequest
from std_msgs.msg import String

# MQTT config
config = configparser.RawConfigParser()
config.read('bridge.conf')
broker = config.get('ha_om_bridge', 'broker')
port   = config.get('ha_om_bridge', 'port')
username = config.get('ha_om_bridge', 'mqtt_username')
password = config.get('ha_om_bridge', 'mqtt_password')
mqtt_control_topic = "mowgli/hactrl"
client_id = f'mowgli-mqtt-{random.randint(0, 100)}'

# OM ROS config
ros_service = "/mower_service/high_level_control"

# exit quicker when roslaunch is stopped
def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


# ROS Service - OpenMower HighLevelControlSrv
def ros_hlc(msg_payload):
    print(f"ha_om_brigde.py: Calling ROS Service `{ros_service}` with payload of: `{msg_payload}`")
    try:
        hlc_request = rospy.ServiceProxy(ros_service, HighLevelControlSrv)
        hlc_request(msg_payload)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# MQTT connect
def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("ha_om_brigde.py: Connected to MQTT Broker!")
        else:
            print("ha_om_brigde.py: Failed to connect to MQTT Broker, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


# MQTT subscribe and receive msg 
def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        print(f"ha_om_brigde.py: Received MQTT msg `{msg.payload.decode()}` from `{msg.topic}` topic")
        payload = msg.payload.decode()
        if payload == "command_start":
            ros_hlc(HighLevelControlSrvRequest.COMMAND_START)
        if payload == "command_home":
            ros_hlc(HighLevelControlSrvRequest.COMMAND_HOME)
        if payload == "command_s1":
            ros_hlc(HighLevelControlSrvRequest.COMMAND_S1)
        if payload == "command_s2":
            ros_hlc(HighLevelControlSrvRequest.COMMAND_S2)

    client.subscribe(mqtt_control_topic)
    client.on_message = on_message


def run():
    signal.signal(signal.SIGINT, signal_handler)
    print(f"ha_om_brigde.py: Waiting for ROS Service `{ros_service}` ...")
    rospy.wait_for_service(ros_service)
    print(f"ha_om_brigde.py: ROS Service `{ros_service}` became available, starting to listen for MQTT messages on `{mqtt_control_topic}`")
    rospy.init_node('ha_om_brigde', anonymous=True)
    # MQTT
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()

if __name__ == '__main__':
    run()
