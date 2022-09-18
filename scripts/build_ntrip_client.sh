#!/bin/bash

cd ~/MowgliRover/
catkin_make --only-pkg-with-deps ntrip_client 
cd -
