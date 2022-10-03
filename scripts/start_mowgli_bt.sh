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
if [ "$1" == "test" ];
then
    echo ""
    echo "======================================================================"
    echo "= starting with test.xml !"
    echo "======================================================================"
    echo ""
    sleep 1
    roslaunch mowgli mowgli_bt_test.launch
else
    roslaunch mowgli mowgli_bt_test.launch
fi
cd -
