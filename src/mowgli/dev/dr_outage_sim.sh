#!/bin/bash


DELAY=3

while true
do
 ./dr_fake_gps_outage.sh
  sleep $DELAY
 ./dr_clear_gps_outage.sh
 sleep $DELAY
done

