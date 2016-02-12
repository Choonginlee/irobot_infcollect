# irobot_infcollect

!requirements

[1] irobot: check the ID of usb and add to ttyUSB0
- lsusb to check ID
- sudo modprobe usbserial vendor=[vendor ID] product=[product ID]
- dmesg | grep tty to check ttyUSB port
- Set ttyUSB1

- install ncurses library (libncurses5-dev libncursesw5-dev)

[2] pgr camera: 
- install flycapture https://www.ptgrey.com/support/downloads

[3] xg1010:
- dmesg | grep tty to check ttyUSB port
- Set tty USB0

!How to compile

Just type command below:
sudo sh ./go

!How to run

mkdir result
sudo ./irobot