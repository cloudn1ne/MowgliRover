#!/bin/bash

echo ""
echo -n "calling /mowgli/Reboot ... "
rosservice call /mowgli/Reboot
echo " OK"
echo -n "waiting 5sec for Mowgli to reboot "
sleep 1 && echo -n "."
sleep 1 && echo -n "."
sleep 1 && echo -n "."
sleep 1 && echo "."
echo -n "restarting ROS Serial ... "
sudo systemctl restart rosserial
echo " OK"
