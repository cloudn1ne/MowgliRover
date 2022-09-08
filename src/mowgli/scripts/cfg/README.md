# Configuration variables for Mowgli

Variables are stored in the onboard SPI Flash
Currently a reboot counter, as well as the IMU (Magentometer) Calibration values are saved.

## Saving/Restoring IMU Calibration

### Example save

```
./save_imu_calibration.sh outdoor_closed
saving mag_bias_x = 0.000024648246530
saving mag_bias_y = -0.000014333906581
saving mag_bias_z = -0.000005657317766
saving mag_dist_00 = 1.012488251753339
saving mag_dist_01 = 0.023911289583521
saving mag_dist_02 = 0.029519032984690
saving mag_dist_10 = 0.023911289583521
saving mag_dist_11 = 1.127282957654589
saving mag_dist_12 = 0.031724548047277
saving mag_dist_20 = 0.029519032984690
saving mag_dist_21 = 0.031724548047277
saving mag_dist_22 = 1.026199680167378
```

### Example restore

As calibration values are only read on bootup you will need restart Mowgli then (./ctrl/reboot.sh)
If you had Mowgli running, you will probably need to restart that too then.

```
./restore_imu_calibration.sh outdoor_closed/
restoring mag_bias_x = 0.000024648246530
restoring mag_bias_y = -0.000014333906581
restoring mag_bias_z = -0.000005657317766
restoring mag_dist_00 = 1.012488251753339
restoring mag_dist_01 = 0.023911289583521
restoring mag_dist_02 = 0.029519032984690
restoring mag_dist_10 = 0.023911289583521
restoring mag_dist_11 = 1.127282957654589
restoring mag_dist_12 = 0.031724548047277
restoring mag_dist_20 = 0.029519032984690
restoring mag_dist_21 = 0.031724548047277
restoring mag_dist_22 = 1.026199680167378
```

## Saving/Restoring individual values

### Write (string) config var 

```
python3 setcfg.py -s -n "joe" -v "blahaha"
```

### Write (byte array) config var

```
python3 setcfg.py -a -n "joe" -v "0x1,0x2,0x3,0xAA,0xBB,0xCC"
```

### Read config var

```
python3 getcfg.py -n joe
```
