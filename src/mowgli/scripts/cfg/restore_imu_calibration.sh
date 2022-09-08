#!/bin/bash

if [ $# -ne 1 ]; then 
    echo "$0 <save_name>"
    exit -1
fi

Dir=$1
declare -a CfgVars=("mag_bias_x" "mag_bias_y" "mag_bias_z" "mag_dist_00" "mag_dist_01" "mag_dist_02" "mag_dist_10" "mag_dist_11" "mag_dist_12" "mag_dist_20" "mag_dist_21" "mag_dist_22" )

if [ ! -d "$Dir" ]; then
  echo "unable to open save_name $1"
  exit -1
fi
for var in ${CfgVars[@]}; do
   val=`cat $Dir/$var`
   echo "restoring $var = $val "
   python3 setcfg.py -q -d -v "$val" -n $var
done

