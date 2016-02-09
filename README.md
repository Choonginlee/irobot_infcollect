# irobot_infcollect

requirement:
irobot
-check the ID of usb and add to ttyUSB0
1. lsusb to check ID
2. sudo modprobe usbserial vendor=[vendor ID] product=[product ID]
3. dmesg | grep tty to check ttyUSB port


