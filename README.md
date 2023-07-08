### Install RPi Pico SDK
In a personal folder called projects (could be other):
```bash
cd ~/projects
mkdir pico
cd pico
git clone -b master https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
cd ..
git clone -b master https://github.com/raspberrypi/pico-examples.git
sudo apt update
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
cd pico-examples
mkdir build
cd build
echo "export PICO_SDK_PATH=~/projects/pico-sdk" >> ~/.bashrc
source ~/.bashrc
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j4

```
### Install OpenOCD and debugger
```bash
cd ~/projects/pico
sudo apt install automake autoconf build-essential texinfo libtool libftdi-dev libusb-1.0-0-dev
git clone https://github.com/raspberrypi/openocd.git --recursive --branch rp2040 --depth=1
cd openocd
./bootstrap
./configure --enable-ftdi --enable-sysfsgpio --enable-bcm2835gpio
make -j4
sudo make install
sudo apt install gdb-multiarch

```
### OpenOCD Setup
Add username to the plugdev group to allow non-sudo access to any USB device:
```bash
echo 'SUBSYSTEM=="usb", MODE="0660", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/00-usb-permissions.rules
udevadm control --reload-rules
sudo usermod -a -G plugdev [username]
```
Log out and back in.

### Debugging session
To debug, start opencd in a seperate terminal in the ~/projects/pico/openocd folder:
```bash
cd ~/projects/pico/openocd
src/openocd -f interface/picoprobe.cfg -f target/rp2040.cfg -s tcl
```

### Current state
## Battery voltage reading
Battery / motor ground is decoupled from digital ground. That is good, however the voltage reading is offset and unreliable because of this. Apparently with proper star grounding the grounds could be connected together without issues. Otherwise the most reliable way of achieving this would be to use a seperate DC/DC converter and ADC on the high-power side, and opto-coupler on the digital link to RP2040. 

## Orientation sensor calibration
A proper procedure to calibrate the BNO055 should be written. Ideally it should be automated when/if the robot can achieve it by itself.

## Serial communication with head
The link between base and head (ROS-based) was not completed yet.

## Stability
Finish test_ctl

## LCD
LCD is not working anymore...
