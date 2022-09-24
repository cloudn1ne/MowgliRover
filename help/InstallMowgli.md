# Install ROS Noetic + Mowgli + OpenMower on Ubuntu 20.04 Server on a Raspi

## Required Items

* Raspi 4
* 32GB or better SD Card
* USB Keyboard
* Mini HDMI to XXX adapter and a monitor
* WLAN credentials and internet connectivity
* A PC running Ubuntu 20.04 (for Magentometer Calibration) or untested by me it should run on the Raspi4 as well.

## Install Ubuntu 20.04 Server 

* install ubuntu 20.04 with imager -> https://www.pragmaticlinux.com/2021/08/install-the-raspberry-pi-imager-on-ubuntu-debian-fedora-and-opensuse/
* RPI Imager download -> https://www.raspberrypi.com/software/
    * Operating System: UBUNTU Server 20.04.5 LTS (RPI 3/4/400) 64 BIT
* Boot up and set pw for user ubuntu (default credentials are UN: ubuntu PW:ubuntu)
* Configure network so you can reach your raspi remotely with a static ip or dhcp static lease -> https://linuxconfig.org/ubuntu-20-04-connect-to-wifi-from-command-line
    Note that i use an IOT WLAN for my garden, and have not tried anything with a Hotspot type setup.
* Connect to your Raspi via ssh, use the 'ubuntu' user

### Configure NTP

Set your local timezone ..

```
sudo timedatectl set-timezone Europe/Vienna
sudo timedatectl set-ntp on
```

### Stop auto updates (from interfering)

```
sudo systemctl stop --force unattended-upgrades.service
```

### Remove snapd

```
sudo snap remove --purge lxd
sudo snap remove --purge core20
sudo snap remove --purge snapd
sudo apt -y remove --autoremove snapd
```

### Update Ubuntu to latest version

```
sudo apt-get update
sudo apt-get -y upgrade
sudo apt-get -y install curl git
```

## Install ROS noetic -> http://wiki.ros.org/noetic/Installation/Ubuntu

### Perform steps: 1.2, 1.3

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

### For step: 1.4 use the ROS-Base (Bare Bones) option

```
sudo apt update
sudo apt install -y ros-noetic-ros-base
```

### Perform step: 1.5

```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/MowgliRover/devel/setup.bash" >> ~/.bashrc
```

### Install ROS packages and other good stuff that Mowgli and OM will need

```
sudo apt-get -y install g++ cpp cmake ros-noetic-tf2-eigen ros-noetic-teleop-twist-keyboard ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher ros-noetic-map-server ros-noetic-rosserial-server ros-noetic-gps-common picocom libraspberrypi-bin  ros-noetic-tf2-geometry-msgs ros-noetic-robot-localization libraspberrypi-dev libraspberrypi0 libpigpiod-if-dev wavemon ros-noetic-rosserial-arduino python3-paho-mqtt openocd python3-rosdep ros-noetic-rtcm-msgs
```

### Add some alias helpers to \~/.bashrc

```
alias depit="rosdep install --from-paths src --ignore-src -r -y"
alias debug="picocom -b 115200 /dev/ttyAMA1"
alias vbat="rostopic echo -n 1 /mowgli/status/v_battery | head -1"
```

### Configure ROS Core IP (in /opt/ros/noetic/setup.bash)

\<your-rover-ip\> is your static ip address for the raspi.
add the export ROS_IP, ROS_MASTER_URI to your setup.bash file

```
export ROS_IP=<your-rover-ip>
export ROS_MASTER_URI=http://localhost:11311

CATKIN_SHELL=bash
```

## Installing Mowgli, OpenMower and dependencies

### Clone MowgliRover Repo

```
cd ~
git clone https://github.com/cloudn1ne/MowgliRover.git
```

### Clone submodules

This will fetch RTKLIB, openmower_ros and all its subrepos

```
cd ~/MowgliRover/
git submodule update --init --recursive
catkin_make
```

### Source your setup.bash files

```
source ~/.bashrc
```

### Install ROS dependencies

```
cd ~/MowgliRover/
sudo rosdep init
rosdep update
depit
source ~/.bashrc
```

### Build Mowgli, OM, RobotLocalization, ...

Note: Create a swap file around 4G in size if you have less than 4G memory or compiles will fail (and take ages). I managed to (test) compile this on a Raspi3 that way, but i have not actually tried to run it with a real bot.

```
cd ~/MowgliRover/scripts
./build_all.sh
```

## Plugin the YF Mainboard with Mowgli usbnode installed via USB

Your YF mainbord needs to run https://github.com/cloudn1ne/Mowgli/tree/main/stm32/ros_usbnode

**You need to run a version released after 20th Sept 2022 as only those will include the required srvs and topics !!!) - it will not work with an older version **

In general its a good idea to update Mowgli (stm32) whenever there is a new release because some feature enhancements on MowgliRover will need new stm32 code (or fixes there). Hopefully a version system will soon be working that will tell you that your Mowgli (stm32) is too old when running MowgliRover.

See https://github.com/cloudn1ne/Mowgli for more information how to do that.

### Install the roscore and rosserial service scripts

```
cd ~/MowgliRover/scripts/systemctl
./install.sh
```

The rosserial script's usbreset expects Mowgli to be connected to /dev/bus/usb/001/002

If thats not the case (check with lsusb) you need to adjust those values. This ensures that rosserial recovers from a reboot/reflash of Mowgli automatically.


## Remove some useless stuff to lighten the load on the Raspi

```
sudo apt-get remove --purge pulseaudio
sudo systemctl disable ModemManager
sudo systemctl stop ModemManager
```

### Reboot 

```
sudo reboot
````

### Install complete

If your Raspi comes back, then congratulations you are now ready to configure Mowgli and Openmower

See [here](ConfigureMowgli.md) for the next steps


