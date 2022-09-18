# MowgliRover

This repo holds the Mowgli/OM files that need to be deployed onto your Raspi.

The various pieces assume that your build matches https://github.com/cloudn1ne/Mowgli/ 

If your build deviates (different IMU, IMU connected to Raspi, etc ..) you will need to adapt the files in this repo.

To get started follow the instructions in [here](help/InstallMowgli.md)

## Main Branch (defaul)

The main branch is mostly a 1:1 Mowgli/OM integration and will only work if you have perfect GPS RTK (fix, not float) reception in your garden.
If thats not true for your garden have a look at the Testing Branch below.

## Testing Branch

Latest developments happen in the testing branch. Usually i tried to checkin only versions that work, but bugs can happen.

To checkout the latest 'testing' branch follow [this howto](https://github.com/cloudn1ne/MowgliRover/blob/testing/help/TestingBranch.md) to switch from a Main branch repo to a testing branch.

## TODO:

* Display GPS-RTK Fix (Quality ?) with YF status Leds during Map Recording, slow down robot if quality diminishes ?
* Improve Dead Reckoning, more testing is needed (sharp turns vs. straight ahead driving)
* FTC PID values, currently only P is used, ID might be helpful on rougher surfaces
* Include patched FTC controller in this repo (better debugging if it fails)
* Include patched mower_logic (better debugging, state display, and changed values for the DockingBehaviour)
* Halt execution of path if Dead Reckoning reaches the configured maximum duration (instead of going to NULL behaviour) and continue when GPS-RTK is re-aquired.

