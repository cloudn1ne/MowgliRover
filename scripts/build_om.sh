#!/bin/bash
cd ~/MowgliRover/
catkin_make --only-pkg-with-deps mower_logic
catkin_make --only-pkg-with-deps mower_utils
catkin_make --only-pkg-with-deps slic3r_coverage_planner
cd -
