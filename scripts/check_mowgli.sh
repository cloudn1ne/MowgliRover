#!/bin/bash
# Mowgli prereq check script v1.0

echo "" 

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
