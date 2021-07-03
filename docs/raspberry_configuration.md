# Raspberry Configuration

This document explains how to configure a fresh raspberry in order to make it run the prototype.

## Installation
Install the latest release of Raspberry Pi OS with [Raspberry Pi Imager](https://www.raspberrypi.org/software/)

> Make sure to install the headless version

## Set up
After you installed the OS onto the SD, we need to set up a few things. 

### SSH
In order to access the raspberry while we develop the prototype, we will use an SSH connection.
To enable SSH, in the SD of raspberry, just create an empty file without extension called `ssh`

> Make sure to place the `ssh` file in the boot partition

Now whenever the raspberry will boot, will be able to access it via terminal by typing:
```bash
ssh pi@raspberrypi
``` 

By default, the password will be **raspberry**. Make sure to change it using the `passwd` command.

### Wi-Fi
In order to connect to a Wi-Fi network, we need to place another file in the boot partition of the SD.
Create a `wpa_supplicant.conf` file in the boot partition, and add the following inside.
```text
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=IT

network={
 ssid="SSID"
 psk="pass"
}
```

> Make sure to replace SSID and pass with the name of the Wi-Fi network and the password
 
### Serial connection
In order to connect the raspberry with the Flight Controller (FC from here), we need to enable the serial port.

Since Raspberry uses bluetooth on the serial port, let's disable it.

```bash
sudo systemctl disable hciuart
```

Then we need to append to variables in the raspberry configuration file. Let's open the configuration file by typing: 
```bash
sudo nano /boot/config.txt
```

Paste at the end of the file these two lines:
```bash
dtoverlay=pi3-disable-bt
enable_uart=1
```

Finally, let's add a new environment variable that will be used by our prototype. 

```bash
echo "ARDU_SERIAL_CONN=serial:///dev/ttyAMA0:57600" > ~/.bashrc
```

Now we can reboot with `sudo shutdown -r now`.

### Software
There's a problem with Raspberry Pi Zero W, so need a workaround to make everything work.

```bash
pip3 uninstall grpcio && sudo apt-get install python3-grpcio
```

Then we can install the package so type:
```bash
pip3 install mavsdk --user~~~~
```