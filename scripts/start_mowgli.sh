#!/bin/bash

echo "loading mowgli_config.sh"
source ~/MowgliRover/src/mowgli/config/mowgli_config.sh

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
