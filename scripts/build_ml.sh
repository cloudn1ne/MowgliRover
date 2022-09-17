#!/bin/bash

cd ~/MowgliRover/
catkin_make --only-pkg-with-deps mower_logic
cd -
