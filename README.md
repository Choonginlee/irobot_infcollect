# irobot_infcollect

!requirements

[1] xg1010:
- dmesg | grep tty to check ttyUSB port
- Set tty USB0

[2] irobot: check the ID of usb and add to ttyUSB1
- lsusb to check ID
- sudo modprobe usbserial vendor=[vendor ID] product=[product ID]
- dmesg | grep tty to check ttyUSB port
- Set ttyUSB1

- install ncurses library (libncurses5-dev libncursesw5-dev)

[3] pgr camera: 
- install flycapture https://www.ptgrey.com/support/downloads


!How to compile

Just type command below:
sudo sh ./go_s or ./go_m

!How to run

mkdir result
sudo ./irobot
