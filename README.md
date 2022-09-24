# MowgliRover

This repo holds the Mowgli/OM files that need to be deployed onto your Raspi.

The various pieces assume that your build matches https://github.com/cloudn1ne/Mowgli/ 

If your build deviates (different IMU, IMU connected to Raspi, etc ..) you will need to adapt the files in this repo.

No matter if you use the Main or Testing branch you will need to install ROS and setup the environment prereqs.
To get started follow the instructions in [here](help/InstallMowgli.md)

## Testing Branch

Latest developments happen in the testing branch. Usually i tried to checkin only versions that work, but bugs can happen.

To checkout the latest 'testing' branch follow [this howto](https://github.com/cloudn1ne/MowgliRover/blob/testing/help/TestingBranch.md) to switch from a 'main' branch to the 'testing' branch.

Note that the 'testing' branch will have updated ./help documentatin where applicable (dont rely on the 'main' help for 'testing')

## TODO:

* Display GPS-RTK Fix (Quality ?) with YF status Leds during Map Recording, slow down robot if quality diminishes ?
* ~~Improve Dead Reckoning, more testing is needed (sharp turns vs. straight ahead driving)~~ 
* FTC PID values, currently only P is used, ID might be helpful on rougher surfaces
* Include patched FTC controller in this repo (better debugging if it fails)
* ~~Include patched mower_logic (better debugging, state display, and changed values for the DockingBehaviour)~~ 
* ~~Halt execution of path if Dead Reckoning reaches the configured maximum duration (instead of going to NULL behaviour) and continue when GPS-RTK is re-aquired~~ ('testing' branch)
* Allow Docking at any time (not just MowBehaviour) and without Undocking first.
* Allow DR only for Docking, Undocking

