### OpenOCD Setup
Add username to the plugdev group to allow non-sudo access to any USB device:
```bash
echo 'SUBSYSTEM=="usb", MODE="0660", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/00-usb-permissions.rules
udevadm control --reload-rules
sudo usermod -a -G plugdev [username]
```
Log out and back in.


To debug, start opencd in a seperate terminal in the ~/pico/openocd folder:
```bash
src/openocd -f interface/picoprobe.cfg -f target/rp2040.cfg -s tcl
```