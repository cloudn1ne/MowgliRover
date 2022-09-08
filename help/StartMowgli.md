# Starting Mowgli

### Prerequisites

1. roscore, rosserial, rosserial_watchdog running
2. Mowgli https://github.com/cloudn1ne/Mowgli/tree/main/stm32/ros_usbnode flashed on the mainboard, connected to the raspi

```
~/MowgliRover/scripts/check_mowgli.sh
```

Output should look something like

```
>> testing for mowgli_config.sh
>> testing systemctl services
>> testing /mowgli/status
>> testing /mowgli/odom


**************************************
* You should be good to run Mowgli ! *
**************************************
```

### Start Mowgli

```
~/MowgliRover/scripts/start_mowgli.sh
```

