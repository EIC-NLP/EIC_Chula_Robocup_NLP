https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/

# Setup Permissions for USB

1. Add rules

`echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="2886", ATTRS{idProduct}=="0018", GROUP="users", MODE="0666"' | sudo tee /etc/udev/rules.d/50-myusb.rules`

`sudo udevadm control --reload-rules && sudo udevadm trigger`

2. Unplug the Mic and plug it back in

# Install libraries

`cd pixel_ring`

`pip install -U -e .`
