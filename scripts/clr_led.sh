#!/bin/bash

echo $1
rosservice call /mowgli/ClrLed "{ led: $1 }"
