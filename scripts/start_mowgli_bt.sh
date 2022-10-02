#!/bin/bash

echo "Loading mowgli_config.sh"
if [ -f ~/MowgliRover/src/mowgli/config/mowgli_config.sh ];
then
 source ~/MowgliRover/src/mowgli/config/mowgli_config.sh
else
 echo "No mowgli_config.sh found at ~/MowgliRover/src/mowgli/config/mowgli_config.sh"
 exit 1
fi

cd ~/MowgliRover
roslaunch mowgli mowgli_bt2.launch
cd -
