#!/usr/bin/env bash

sudo echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6001\", MODE:=\"0666\", SYMLINK+=\"jsk_board\"" >> /etc/udev/rules.d/70-ttyusb.rules

echo "configuration completed."
