#!/bin/bash
# Mowgli prereq check script v1.0


WS_BASE=~/MowgliRover
declare -a PKGS_REQUIRED=("mowgli" "robot_localization" "mower_msgs" "mower_logic" "mower_map" "slic3r_coverage_planner" "joy" "teleop_twist_joy" "twist_mux" "mbf_costmap_nav" "ublox_gps" "imu_filter_madgwick")

### COMPATIBLITY MATRIX (stm32 vs mowglirover)

check_compatibility()
{
	MINVER="1.0.1"
	if [[ "$MOWGLI_VER" < "$MINVER" ]]; 
	then
		echo ""
		echo "ERROR: You version of Mowgli(stm32) (v$MOWGLI_VER) is not supported by MowgliRover, you need at least v$MINVER"
		echo "       Please update your STM32 code !"
		exit -1
	fi
	echo "   * VERSION match [OK] !"
}


### MAIN

echo "" 

###########################################################
# check workspace base (WS_BASE)
###########################################################
if [ ! -e "$WS_BASE/.catkin_workspace" ];
then
	echo ""
   	echo "ERROR: \$WS_BASE=$WS_BASE is incorrect, please set the right path to the workspace in this script"
   	exit -1
fi

###########################################################
# check if MowgliRover version matches Mowgli-openmower_ros
###########################################################
echo ">> checking version compatiblity"
if [ ! -e "$WS_BASE/VERSION" ];
then
	echo ""
	echo "ERROR: no VERSION file for MowgliRover found"
	exit -1
else
	MOWGLIROVER_VERSION=`cat $WS_BASE/VERSION`
fi
echo "   * MowgliRover: v$MOWGLIROVER_VERSION"
if [ ! -e "$WS_BASE/src/Mowgli-open_mower_ros/VERSION" ];
then
	echo ""
        echo "ERROR: no VERSION file for Mowgli-open_mower_ros found"
        exit -1
else
        MOWGLIOM_VERSION=`cat $WS_BASE/src/Mowgli-open_mower_ros/VERSION`
fi
echo "   * Mowgli-open_mower_ros: v$MOWGLIOM_VERSION"
if [ "$MOWGLIOM_VERSION" != "$MOWGLIROVER_VERSION" ];
then
	echo ""
	echo "ERROR: Version mismatch, please make sure you have the latest version of both repos checked out !"
	echo ""
	echo "  To checkout the latest version of Mowgli-open_mower_ros use:"
	echo "  cd $WS_BASE/src/Mowgli-open_mower_ros;git checkout main;git pull"
	echo ""
	echo "  To checkout the latest version of MowgliRover use:"
	echo "  cd $WS_BASE;git pull" 
  	echo ""
	echo "  Do not forget to recompile your code ./scripts/build_all.sh after updating the repos !"
	exit -1
else
	echo "   * VERSION match [OK] !" 
fi

# check if all packages required are installed/compiled
###########################################################
echo ""
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
# testing for usbreset 
###########################################################
if [ ! -f "/usr/bin/usbreset" ];
then
 echo "ERROR: /usr/bin/usbreset not found"
 exit -1
fi

###########################################################
# testing for mowgli_config.sh
###########################################################
echo ""
echo ">> testing for mowgli_config.sh"
if [ ! -f ~/MowgliRover/src/mowgli/config/mowgli_config.sh ];
then
 echo "ERROR: no mowgli_config.sh found, check ~/MowgliRover/src/mowgli/config/ for example files"
 exit -1
fi

###########################################################
# check if we have the services running
###########################################################
echo ""
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
echo ""
echo ">> testing /mowgli/status"
if ( ! rostopic info /mowgli/status > /dev/null 2>&1 ) ;
then
	 echo "ERROR: no /mowgli/status topic, is Mowgli connected and actually working ?"
	 echo "(check debug output via serial console by running 'debug')"
	 exit -1
else
	# check version
	echo "   * gathering version information ... stand by"
	MOWGLI_MAJ=`rostopic echo -n 1 /mowgli/status/sw_ver_maj | head -1`
	MOWGLI_BRA=`rostopic echo -n 1 /mowgli/status/sw_ver_bra | head -1`
	MOWGLI_MIN=`rostopic echo -n 1 /mowgli/status/sw_ver_min | head -1`
	MOWGLI_VER="$MOWGLI_MAJ.$MOWGLI_BRA.$MOWGLI_MIN"
	echo "   * Mowgli(STM32) version: $MOWGLI_VER"
	check_compatibility

fi;

###########################################################
# checking /mowgli/odom
###########################################################
echo ""
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
echo ""
