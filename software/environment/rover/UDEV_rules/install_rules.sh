#!/bin/bash
sudo cp 99-rover-cameras.rules /etc/udev/rules.d/.
sudo cp 99-rover-usb-serial.rules /etc/udev/rules.d/.
sudo reboot
