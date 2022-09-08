#!/bin/bash

if [ $# -ne 1 ]; then 
    echo "$0 <save_name>"
    exit -1
fi

Dir=$1
declare -a CfgVars=("mag_bias_x" "mag_bias_y" "mag_bias_z" "mag_dist_00" "mag_dist_01" "mag_dist_02" "mag_dist_10" "mag_dist_11" "mag_dist_12" "mag_dist_20" "mag_dist_21" "mag_dist_22" )

if [ ! -d "$Dir" ]; then
 mkdir -p $1
fi
for var in ${CfgVars[@]}; do
   echo -n "saving $var = "
   val=`python3 getcfg.py -q -n $var`
   echo $val
   echo $val > $Dir/$var
done

