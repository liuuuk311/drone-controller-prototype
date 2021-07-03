#!/bin/bash

echo "Setup Started!"

# Updating and Installing
sudo apt update
sudo apt upgrade -y

sudo apt install python3-pip python3-dev 

# Python stuff
```
pip3 uninstall grpcio
sudo apt-get install python3-grpcio
```
pip3 install mavsdk --user

# Config
echo "Preparing to config raspberry..."
sudo echo "enable_uart=1" >> /boot/config.txt
sudo echo "dtoverlay=disable-bt" >> /boot/config.txt

echo "Configuration completed... Rebooting"
sudo reboot now