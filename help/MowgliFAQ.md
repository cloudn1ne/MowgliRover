```
[527576.920668] usb 1-1.3: reset full-speed USB device number 119 using xhci_hcd
[527576.920867] usb 1-1.3: Device not responding to setup address.
[527577.128810] usb 1-1.3: Device not responding to setup address.
[527577.336642] usb 1-1.3: device not accepting address 119, error -71
[527577.345155] usb 1-1.3: USB disconnect, device number 119
[527577.428816] usb 1-1.3: new full-speed USB device number 121 using xhci_hcd
[527577.508825] usb 1-1.3: device descriptor read/64, error -32
[527577.700900] usb 1-1.3: device descriptor read/64, error -32
[527577.896651] usb 1-1.3: new full-speed USB device number 122 using xhci_hcd
[527577.980869] usb 1-1.3: device descriptor read/64, error -32
[527578.172831] usb 1-1.3: device descriptor read/64, error -32
```

USB errors in syslog, and Mowgli no longer emitting ROS topics - can happen during flashing.

Reset the USBHub with 

```
sudo /usr/bin/usbreset "USB2.0 Hub"
```

```
[ WARN] [1664017729.750420769]: Costmap2DROS transform timeout. Current time: 1664017729.7503, global_pose stamp: 1664017578.3951, tolerance: 0.3000
[ WARN] [1664017729.941969716]: mowgli_proxy: Dropped GPS Update due to carrier phase range solution not being FIXED (gps quality might not be good enough)1
[ WARN] [1664017730.047740936]: mowgli_proxy: Dead Reckoning requested but we dont have a recent GPS RTK fix - EKF is not up to date !
```

This means that there is currently no 3D Fix, and Dead Reckoning is unavailable because we either never had a proper fix, or the DR max runtime has already been reached.

```
[move_base_legacy_relay-9] process has died [pid 8421, exit code 1, cmd /opt/ros/noetic/lib/mbf_costmap_nav/move_base_legacy_relay.py __name:=move_base_legacy_relay __log:=/home/ubuntu/.ros/log/012bfcf6-3020-11ed-90f5-31c7ae61b335/move_base_legacy_relay-9.log].
log file: /home/ubuntu/.ros/log/012bfcf6-3020-11ed-90f5-31c7ae61b335/move_base_legacy_relay-9*.log
```

If it takes too long to get a inital GPS fix (freshly rebooted for example) then move_base_legacy_relay will die. 
As it is set to auto restart that shouldnt be a problem. You can ignore that.

```
[ERROR] [1662716545.197170671]: U-blox: received NACK: 0x06 / 0x01
```

Has no impact, happens everytime ublox gets started

```
 INFO] [1664017574.840651931]: mowgli_proxy: pubOdometry(/odom) [ GPS RTK-Fixed ] x = -7.71m, y = 4.50m, yaw = 76.32deg
[ WARN] [1664017575.342048094]: mowgli_proxy: Dropped GPS Update due to carrier phase range solution not being FIXED (gps quality might not be good enough)1
[ WARN] [1664017575.342292296]: mowgli_proxy: doDeadReckoning: activated
[ INFO] [1664017575.394623983]: mowgli_proxy: pubOdometry(/odom) [ DR ] x = -7.71m, y = 4.51m, yaw = 131.09deg
[ WARN] [1664017576.342683699]: mowgli_proxy: Dropped GPS Update due to carrier phase range solution not being FIXED (gps quality might not be good enough)1
[ INFO] [1664017576.395625807]: mowgli_proxy: pubOdometry(/odom) [ DR ] x = -7.71m, y = 4.50m, yaw = 121.65deg
[ WARN] [1664017577.442026270]: mowgli_proxy: Dropped GPS Update due to carrier phase range solution not being FIXED (gps quality might not be good enough)1
[ INFO] [1664017577.494524753]: mowgli_proxy: pubOdometry(/odom) [ DR ] x = -7.71m, y = 4.50m, yaw = 109.19deg
[ WARN] [1664017578.449364894]: mowgli_proxy: Dropped GPS Update due to carrier phase range solution not being FIXED (gps quality might not be good enough)1
[ERROR] [1664017578.449708039]: mowgli_proxy: doDeadReckoning: Maximum Dead Reckoning (3 sec) time reached
[ WARN] [1664017578.541109513]: mowgli_proxy: Dead Reckoning requested but we dont have a recent GPS RTK fix - EKF is not up to date !
[ WARN] [1664017578.750453024]: Costmap2DROS transform timeout. Current time: 1664017578.7503, global_pose stamp: 1664017578.3951, tolerance: 0.3000
[ WARN] [1664017579.415085680]: #### om_mower_logic: setMowerEnabled(0) call
[ WARN] [1664017579.427411331]: Most recent robot pose is 1.03225s old (tolerance 1s)
[ERROR] [1664017579.427589311]: Could not get the robot pose in the global frame. - robot frame: "base_link"   global frame: "map
[FATAL] [1664017579.428095344]: Internal error: Unknown error thrown by the plugin: Could not get the robot pose
[ERROR] [1664017579.430700543]: MowingBehavior: (FIRST POINT) - Could not reach goal (first point). Planner Status was: 5
[ WARN] [1664017579.430843727]: MowingBehavior: (FIRST POINT) - Attempt 2 / 3 Making a little pause ...
[ INFO] [1664017579.430934967]: MowingBehavior: PAUSED (6.833e-06s) (waiting for /odom)
```

Mowgli (while mowing) had a GPS RTK fix, but then lost it - Dead Reckoning (DR) was activated, and ran for 3 seconds after which the Bot stopped (PAUSED).


```
[ INFO] [1664017797.445206285]: mowgli_proxy: pubOdometry(/odom) [ GPS RTK-Float ] x = -7.42m, y = 4.53m, yaw = 101.73deg
[ INFO] [1664017797.503227846]: MowingBehavior: PAUSED (218.072s) (waiting for /odom)
[ INFO] [1664017798.503565267]: MowingBehavior: PAUSED (219.073s) (waiting for /odom)
[ INFO] [1664017798.543709332]: mowgli_proxy: pubOdometry(/odom) [ GPS RTK-Float ] x = -7.41m, y = 4.54m, yaw = 102.04deg
[ INFO] [1664017799.503863948]: MowingBehavior: PAUSED (220.073s) (waiting for /odom)
[ INFO] [1664017799.552687329]: mowgli_proxy: pubOdometry(/odom) [ GPS RTK-Float ] x = -7.42m, y = 4.54m, yaw = 102.06deg
[ INFO] [1664017800.504171612]: MowingBehavior: PAUSED (221.073s) (waiting for /odom)
[ INFO] [1664017800.643030800]: mowgli_proxy: pubOdometry(/odom) [ GPS RTK-Fixed ] x = -7.36m, y = 4.47m, yaw = 102.06deg
[ INFO] [1664017801.504546554]: MowingBehavior: CONTINUING
````

Mowgli was paused for 217 seconds, and then a GPS RTK Fixed (not the GPS RTK-Float !) was re-aquired, which after 5 successfull messages will make Mowgli CONTINUE on its mow path. Note that the pubOdometry messages only how every second (but GPS messages arrive at 10Hz)


```
[WARN] [1664058663.218971]: Found packet, but checksums didn't match
[WARN] [1664058663.223441]: Expected Checksum: 0xE80C05
[WARN] [1664058663.228792]: Actual Checksum:   0x556933
```

The ntrip_client encountered a RTCM frame with an invalid checksum - should happen only occasionally - no impact afaik

