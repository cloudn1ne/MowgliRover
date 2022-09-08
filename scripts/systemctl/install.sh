#!/bin/bash

echo "installing UDEV rules"
sudo cp 50-mowgli.rules /etc/udev/rules.d
echo "restarting UDEV"
sudo udevadm control --reload
echo "installing systemctl services for roscore and rosserial"
sudo cp *.service /lib/systemd/system/
sudo ln -s /lib/systemd/system/roscore.service /etc/systemd/system/roscore.service
sudo ln -s /lib/systemd/system/rosserial.service /etc/systemd/system/rosserial.service
sudo ln -s /lib/systemd/system/rosserial_watchdog.service /etc/systemd/system/rosserial_watchdog.service
sudo systemctl daemon-reload

echo "starting roscore, rosserial, rosserial_watchdog"

sudo systemctl enable roscore
sudo systemctl start roscore
sudo systemctl enable rosserial
sudo systemctl start rosserial
sudo systemctl enable rosserial_watchdog
sudo systemctl start rosserial_watchdog

sudo systemctl status --no-pager roscore
sudo systemctl status --no-pager rosserial
sudo systemctl status --no-pager rosserial_watchdog
