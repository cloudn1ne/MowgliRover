#!/bin/bash
cd ~/MowgliRover/
./scripts/build_om.sh
./scripts/build_robot_localization.sh
./scripts/build_mowgli.sh
./scripts/build_ntrip_client.sh
cd -
