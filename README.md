# irobot_infcollect

!requirements

[1] irobot: check the ID of usb and add to ttyUSB0
- lsusb to check ID
- sudo modprobe usbserial vendor=[vendor ID] product=[product ID]
- dmesg | grep tty to check ttyUSB port

- install ncurses library (libncurses5-dev libncursesw5-dev)

[2] pgr camera: 
- install flycapture https://www.ptgrey.com/support/downloads

[3] xg1010:
- blabla


!How to compile

[1] irobot:
- gcc irobot.c -o [Output File Name] -lncurses

[2] pgr camera:
- gcc pgr.c -o [Output File Name] -lflycapture-c

[3] xg1010:
- 
