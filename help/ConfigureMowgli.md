# Configure Mowgli

## Create inital config file from example template

(only do this once or you lose your settings)

```
cp ~/MowgliRover/src/mowgli/config/mowgli_config.sh.example ~/MowgliRover/src/mowgli/config/mowgli_config.sh
```

## Edit mowgli_config.sh

This assumes you have a Mowgli Build, if you have different hardware mounting then you probably need to ajust MOWGLI_GPS_ANTENNA_OFFSET, MOWGLI_IMU_OFFSET differently.

1. Set your OM_NTRIP_* settings to match your NTRIP provider

2. Set MOWGLI_DATUM_LAT and MOWGLI_DATUM_LONG

I use the position when the robot is in the docking station and charging (if your GPS reception permits) this will center the map on the docking station,
which allows for an easy check in rviz if all is right before mowing.

You can obtain this position by running something like:

```
cd ~/MowgliRover
source src/mowgli/config/mowgli_config.sh
roslaunch mowgli gps_rtk.launch
```
and then in another console check for a

1. a good FIX with 

```
rostopic echo /ublox/navpvt
```

2. when the flags value is showing 131 it means you have a good fix.

At this point you can then use the values of "lon" and "lat" for the MOWGLI_DATUM_LAT/MOWGLI_DATUM_LONG settings (just add a dot after the first 2 numbers)


## Edit bridge.conf (if you plan to use HomeAssistant)

Supply host, username, password to ~/MowgliRover/src/mowgli/scripts/bridge.conf

This will be used by the 2 scripts to translate between ROS topics and MQTT topics

## Calibrate Mowglis Magnetometer

Check [here](https://github.com/cloudn1ne/MowgliBase/blob/main/help/Setup.md) on howto setup a ROS Linux Host to perform this.

Alternatively you could checkout the repo onto the Raspi and try there, i have however not tried that yet (let me know if it works)

## Next

see [here](StartMowgli.md) for how to startup Mowgli and OpenMower

