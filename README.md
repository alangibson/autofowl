# autofowl
Automatic Chicken Coop Door

## Installation

### Generic Linux Installation

    curl -O https://downloads.arduino.cc/arduino-1.6.13-linux64.tar.xz
    tar -xvf arduino-1.6.13-linux64.tar.xz
    sudo mv arduino-1.6.13 /opt
    cd /opt/arduino-1.6.13/
    chmod +x install.sh
    ./install.sh

### Install Arduino IDE on Ubuntu

    sudo apt-add-repository ppa:ubuntu-desktop/ubuntu-make
    sudo apt-get update
    sudo apt-get install ubuntu-make
    sudo umake ide arduino

### Install AccelStepper Library

    curl -O http://www.airspayce.com/mikem/arduino/AccelStepper/AccelStepper-1.53.zip
    unzip AccelStepper-1.53.zip
    sudo mv AccelStepper /opt/arduino-1.6.13/libraries/

### Install RTClib

    curl -o RTCLib.zip https://codeload.github.com/adafruit/RTClib/zip/master
    unzip RTCLib.zip
    sudo mv RTClib-master /opt/arduino-1.6.13/libraries/

## Usage

Upload sketch

    /opt/arduino-1.6.13/arduino --board arduino:avr:uno --port /dev/ttyACM0 --upload arduino/autofowl.ino

Listen to serial

    cu -l /dev/ttyACM* -s 9600
