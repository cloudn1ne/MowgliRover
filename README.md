# MowgliRover

This repo holds the Mowgli/OM files that need to be deployed onto your Raspi.

The various pieces assume that your build matches https://github.com/cloudn1ne/Mowgli/ 

If your build deviates (different IMU, IMU connected to Raspi, etc ..) you will need to adapt the files in this repo.

No matter if you use the Main or Testing branch you will need to install ROS and setup the environment prereqs.

To get started follow the instructions in [here](help/InstallMowgli.md)

## Testing Branch

Latest developments happen in the testing branch. Usually i tried to checkin only versions that work, but bugs can happen.

To checkout the latest 'testing' branch follow [this howto](https://github.com/cloudn1ne/MowgliRover/blob/testing/help/TestingBranch.md) to switch from a 'main' branch to the 'testing' branch.

Note that the 'testing' branch will have updated ./help documentation where applicable.

## Legacy Branch

The legacy branch is the inital commit that is mostly compatible with a default openmower_ros installation as of Aug/Sept 2022.
It requires perfect GPS-RTK reception.

## Known deficiencies:

  * PAUSE/CONTINUE only works when in Mowing behaviour, not during Docking or Undocking or AreaRecording - so you need RTK GPS during those phases or weird stuff can     happen.
  * rosserial_python sometimes crashes which is noticeable when the bot loses orientation (the IMU topic update rate is too low) - It seems to be a buffering problem that can be fixed by killing the rosserial process on the raspi, which will get restarted by the watchdog process again. Root cause needs to be checked - it happens infrequently usualy after long uptimes.
  * There are no recovery behaviours - if the bot goes into PAUSE mode because of lack of GPS Quality it will sit there potentially until battery runs out. However i have never had it sit for more than 20mins, usually a RTK fix comes back that allows to drive on. Your mileage may vary by GPS quality ;)
 

## TODO:

* ~~Display GPS-RTK Fix (Quality ?) with YF status Leds during Map Recording, slow down robot if quality diminishes ?~~
* ~~Improve Dead Reckoning, more testing is needed (sharp turns vs. straight ahead driving)~~ 
* FTC PID values, currently only P is used, ID might be helpful on rougher surfaces
* Include patched FTC controller in this repo (better debugging if it fails)
* ~~Include patched mower_logic (better debugging, state display, and changed values for the DockingBehaviour)~~ 
* ~~Halt execution of path if Dead Reckoning reaches the configured maximum duration (instead of going to NULL behaviour) and continue when GPS-RTK is re-aquired~~ ('testing' branch)
* Allow Docking at any time (not just MowBehaviour) and without Undocking first.
* Allow DR only for Docking, Undocking

