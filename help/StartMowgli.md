# Starting Mowgli

### Prerequisites

1. roscore, rosserial, rosserial_watchdog running
2. Mowgli https://github.com/cloudn1ne/Mowgli/tree/main/stm32/ros_usbnode flashed on the mainboard, connected to the raspi

```
~/MowgliRover/scripts/check_mowgli.sh
```

Output should look something like

```
>> checking version compatiblity
   * MowgliRover: v0.9.2
   * Mowgli-open_mower_ros: v0.9.2
   * VERSION match [OK] !

>> testing for required ROS packages
   * checking for ROS Package 'mowgli' [OK]
   * checking for ROS Package 'robot_localization' [OK]
   * checking for ROS Package 'mower_msgs' [OK]
   * checking for ROS Package 'mower_logic' [OK]
   * checking for ROS Package 'mower_map' [OK]
   * checking for ROS Package 'slic3r_coverage_planner' [OK]
   * checking for ROS Package 'joy' [OK]
   * checking for ROS Package 'teleop_twist_joy' [OK]
   * checking for ROS Package 'twist_mux' [OK]
   * checking for ROS Package 'mbf_costmap_nav' [OK]
   * checking for ROS Package 'ublox_gps' [OK]
   * checking for ROS Package 'imu_filter_madgwick' [OK]

>> testing for mowgli_config.sh

>> testing systemctl services

>> testing /mowgli/status
   * gathering version information ... stand by
   * Mowgli(STM32) version: 1.0.0
   * VERSION match [OK] !

>> testing /mowgli/odom


**************************************
* You should be good to run Mowgli ! *
**************************************
```

### Start Mowgli

```
~/MowgliRover/scripts/start_mowgli.sh
```

