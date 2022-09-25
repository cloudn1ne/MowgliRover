#!/bin/bash

echo $1
rosservice call /mowgli/SetLed "{ led: $1 }"
