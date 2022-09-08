#!/bin/bash

cd ~/MowgliRover/lib/RTKLIB/app/consapp/str2str/gcc/
make
sudo make install
ls -l /usr/local/bin/str2str
cd -
