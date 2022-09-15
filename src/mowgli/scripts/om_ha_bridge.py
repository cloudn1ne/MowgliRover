#!/usr/bin/python3
# Mowgli/OpenMower OM to HA Bridge V 1.0
# (c) Georg Swoboda <cn@warp.at> 2022
#
# https://github.com/cloudn1ne/Mowgli
#
#
# v1.0: inital release
# v1.1: added config file engine
#
#
import os
import random
import time
import sys
import configparser
import rospy
from paho.mqtt import client as mqtt_client
from mowgli.msg import status
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

# MQTT config
config = configparser.ConfigParser()
config.read(os.path.join(os.path.dirname(__file__), 'bridge.conf'))
broker = config.get('ha_om_bridge', 'broker')
port   = config.getint('ha_om_bridge', 'port')
username = config.get('ha_om_bridge', 'mqtt_username')
password = config.get('ha_om_bridge', 'mqtt_password')
mqtt_control_topic = config.get('ha_om_bridge', 'mqtt_control_topic')
mqtt_current_behaviour_topic  = config.get('ha_om_bridge', 'mqtt_current_behaviour_topic')
frequency = config.getint('ha_om_bridge', 'frequency')
client_id = f'mowgli-mqtt-{random.randint(0, 100)}'
print(f"om_ha_bridge.py: broker {username}@{broker}:{port}")
print(f"om_ha_bridge.py: update frequency: 1/{frequency}")
print(f"om_ha_bridge.py: mqtt_control_topic = {mqtt_control_topic}")
print(f"om_ha_bridge.py: mqtt_current_behaviour_topic = {mqtt_current_behaviour_topic}")


# nth skip var
nth = 0
gps_nth = 0

# OM ROS config
ros_topic = "/mower_logic/current_state"
mowgli_topic = "/mowgli/status"
gps_topic = "/ublox/fix"

# ROS Topic Callback (mower/current_behaviour)
def ros_callback(data):
    print(f"om_ha_bridge.py: Relaying Current Behaviour `{data.data}` to MQTT")
    client = connect_mqtt()
    result = client.publish(mqtt_current_behaviour_topic, data.data)
    status = result[0]
    client.disconnect()
    if status != 0:
      print(f"Failed to send MQTT message to topic {mqtt_current_behaviour_topic}") 

def mqtt_pub(topic, val):
    client = connect_mqtt()
    result = client.publish(topic, val)
    status = result[0]
    client.disconnect()
    if status != 0:
       print(f"Failed to send MQTT message to topic {topic}")

# GPS Topic Callback (ublox/fix)
def gps_callback(data):
    global gps_nth

    # only report every 10th message (=1/sec)
    if (gps_nth < 10):
        gps_nth = gps_nth +1
        return

    gps_nth = 0
    # map GPS vars to mqtt topics
    mqtt_pub("mowgli/lat", data.latitude)
    mqtt_pub("mowgli/lon", data.longitude)

# ROS Topic Callback (mowgli/status)
def mowgli_callback(data):
    global nth
    global frequency

    # only report every 10th message (=1/sec)
    if (nth < frequency):
        nth = nth +1
        return
    
    nth = 0
    # map ROS vars to mqtt topics
    v_battery_str = f"{data.v_battery:.2f}"
    mqtt_pub("mowgli/v_battery", v_battery_str)
    v_charge_str = f"{data.v_charge:.2f}"
    mqtt_pub("mowgli/v_charge", v_charge_str)
    i_charge_str = f"{data.i_charge:.2f}"
    charge_pwm_str = f"{data.charge_pwm}"
    mqtt_pub("mowgli/charge_pwm", charge_pwm_str)
    mqtt_pub("mowgli/i_charge", i_charge_str)
    mqtt_pub("mowgli/is_charging", data.is_charging)


# MQTT connect
def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("om_ha_bridge.py: Connected to MQTT Broker!")
        else:
            print("om_ha_bridge.py: Failed to connect to MQTT Broker, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def run():
    print("om_ha_bridge.py: started")
    rospy.init_node('om_ha_bridge', anonymous=True)
    rospy.Subscriber(ros_topic, String, ros_callback)
    rospy.Subscriber(mowgli_topic, status, mowgli_callback)
    rospy.Subscriber(gps_topic, NavSatFix, gps_callback)
    # ROS LOOP
    rospy.spin() 

if __name__ == '__main__':
    run()
