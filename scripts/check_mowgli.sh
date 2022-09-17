#!/bin/bash
# Mowgli prereq check script v1.0

declare -a PKGS_REQUIRED=("mowgli" "robot_localization" "mower_msgs" "mower_logic" "mower_map" "slic3r_coverage_planner" "joy" "teleop_twist_joy" "twist_mux" "mbf_costmap_nav" "ublox_gps" "imu_filter_madgwick")


echo "" 

###########################################################
# check if all packages required are installed/compiled
###########################################################
echo ">> testing for required ROS packages"
for i in "${PKGS_REQUIRED[@]}"
do
   echo -n "   * checking for ROS Package '$i'"
   rospack find "$i"  > /dev/null 2>&1
   if [ $? -ne 0 ]; then
	echo ""
	echo "ERROR: ROS package '$i' not found. Either install it (apt-get) or compile (./scripts/build_all.sh)"
	exit -1
   else
	echo " [OK]"
   fi
done


###########################################################
# testing for mowgli_config.sh
###########################################################
echo ">> testing for mowgli_config.sh"
if [ ! -f ~/MowgliRover/src/mowgli/config/mowgli_config.sh ];
then
 echo "ERROR: no mowgli_config.sh found, check ~/MowgliRover/src/mowgli/config/ for example files"
 exit -1
fi

###########################################################
# check if we have the services running
###########################################################
echo ">> testing systemctl services"
if ( ! systemctl is-active --quiet roscore ) ;
then
 echo "ERROR: roscore service is not running"
 exit -1
fi;

if ( ! systemctl is-active --quiet rosserial ) ;
then
 echo "ERROR: rosserial service is not running"
 exit -1
fi;

if ( ! systemctl is-active --quiet rosserial_watchdog ) ;
then
 echo "ERROR: rosserial_watchdog service is not running"
 exit -1
fi;

###########################################################
# checking /mowgli/status
###########################################################
echo ">> testing /mowgli/status"
if ( ! rostopic info /mowgli/status > /dev/null 2>&1 ) ;
then
 echo "ERROR: no /mowgli/status topic, is Mowgli connected and actually working ?"
 echo "(check debug output via serial console by running 'debug')"
 exit -1
fi;

###########################################################
# checking /mowgli/odom
###########################################################
echo ">> testing /mowgli/odom"
if ( ! rostopic info /mowgli/odom > /dev/null 2>&1 ) ;
then
 echo "ERROR: no /mowgli/odom topic, is Mowgli connected and actually working ?"
 echo "(check debug output via serial console by running 'debug')"
 exit -1
fi;


echo ""
echo ""
echo "**************************************"
echo "* You should be good to run Mowgli ! *"
echo "**************************************"
