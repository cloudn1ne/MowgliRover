#!/bin/bash

echo "Loading mowgli_config.sh"
if [ -f ~/MowgliRover/src/mowgli/config/mowgli_config.sh ];
then
 source ~/MowgliRover/src/mowgli/config/mowgli_config.sh
else
 echo "No mowgli_config.sh found at ~/MowgliRover/src/mowgli/config/mowgli_config.sh"
 exit 1
fi

~/MowgliRover/scripts/reset_gps.sh
echo -n "waiting 5secs for GPS to get back online ["
sleep 1
echo -n "x"
sleep 1
echo -n "x"
sleep 1
echo -n "x"
sleep 1
echo -n "x"
sleep 1
echo -n "x]"

cd ~/MowgliRover
roslaunch mowgli mowgli.launch
cd -
