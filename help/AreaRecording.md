# Area Recording
## Bag File

Bag is: /home/ubuntu/.ros/map.bag
This is where openmower has its map stored that is then presented via the MapServer

To start over fresh, remove the map.bag file before starting Mowgli. It will then go into AREA_RECORDING mode automatically.


## How it works

Position the Bot and follow this sequence - more details are in the discord channel and/or Clemens' Videos

 B start to record
 B end to record
 X (1st) set approach point
 X (2nd) set stop point (shoule be docking_distance away from charge prongs)
 Y save area after (2nd X)

## Existing AREA_RECORDING mode

After you have recorded your map, either restart Mowgli or press the HOME button (via script 'press_home.sh', or HA)
