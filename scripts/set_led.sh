#!/bin/bash

if [ $# -lt 1 ];
then 
   echo ""
   echo "$0 <led_number> <chirp> <clearall>"
   echo ""
   echo " <led_number> 1-18 for YF500C"
   echo " <chirp> 1 or 0"
   echo " <clearall> 1 or 0"
   echo ""
   exit -1
fi

v=$1
# chirp flag
if [ "$2" == "1" ];
then
  v=$((v + 128))
fi
# chirp flag
if [ "$3" == "1" ];
then
  v=$((v + 64))
fi

echo "sending $v"
rosservice call /mowgli/SetLed "{ led: $v }"
