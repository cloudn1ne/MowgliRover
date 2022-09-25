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

## Exiting AREA_RECORDING mode

After you have recorded your map, either restart Mowgli or press the HOME button (via script 'press_home.sh', or HA)

## GPS Quality

The "Monday" LED is illuminated when there is a proper GPS RTK Fix (not Float) - if the status changes Mowgli will chirp. You should not attempt to record a map if you dont have the LED illuminated and thus a proper GPS RTK Fix.

## Docking Tips

I usually press the 1st X about 2m from the Docking station (in a straight line, facing towards the charger) and the 2nd X when the front of Mowgli is at the threshold where the plastik starts (but not on it yet) - then a docking distance of 0.85m seems to work pretty reliable. (I do have RTK-GPS coverage around my dock)

