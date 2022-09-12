# Enable Dead Reckoning (testing branch)

## State

Note that this code is pre-alpha at best, im still experimenting a long and stuff might change in the future.

When Mowgli detects a loss in GPS quality (floating vs. fixed solution), precision (>5cm), or hopping (more then 10cm) it will switch to an EKF robot_localization odometry provider.

This EKF provider is fed with updates from GPS (x/y) and IMU (heading) whenever good (see above) data is available.

So short outages or GPS quality issues should be no problem for Mowgli. I managed to get 5-15cm precision on 5m driving in a straight line with that dead reckoning method.

## Setup

Install the latest Mowgli (str32) version that provides the required TF and /mowlgi/odom topic.
Any version older than 11th Sept should work.

## Check version (TF and /mowgli/odom topic is needed)

```
rostopic echo -n 1 /mowgli/odom |grep frame
```

rostopic needs to show the below frame_id and child_frame_id otherwise you are running an too old version of Mwogli (stm32).

```
frame_id: "odom_dr"
child_frame_id: "base_link_dr"
```

## Add new mowgli_config.sh options

*MOWGLI_DR_MAX_DURATION_SEC* = How long Mowlgi will drive with dead reckoning only (no valid GPS fix)
*MOWGLI_DR_UPDATE_INTERVAL* = How often Mowgli will update the EKF state with the current valid GPS fix and IMU heading.

Default settings to add to ~/MowgliRover/src/mowlgi/config/mowgli_config.sh

```
# DR settings
export MOWGLI_DR_MAX_DURATION_SEC=20
export MOWGLI_DR_UPDATE_INTERVAL=1.0
```

For now we want OM to ignore GPS errors (convariance to big when we run on DR)

```
# Ignore GPS errors (we handle that in mowgli_proxy)
export OM_IGNORE_GPS_ERRORS=true
```


