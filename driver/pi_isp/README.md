This repository contains a precompiled version of avrdude (source from https://github.com/kcuzner/avrdude) 
which can be used to flash arduino bootloaders using the Raspberry Pi GPIO pins.

Installation
============

Run

    sudo apt-get install arduino
    git clone https://bitbucket.org/DawnRobotics/pi_isp.git
    cd pi_isp

Connecting your Arduino Compatible Board to the Raspberry Pi
============================================================

For safety, it's recommended that you make these connections with the power off.

To power the Arduino compatible board, the simplest option is to connect it to the Pi using a USB cable.

Now connect up the 4 programming wires. **Important** If your Arduino compatible board runs at 5V (this
is true of the majority of Arduino compatible boards) then you must level shift the programming lines
to the 3.3V of the Pi, otherwise you will damage the Pi. A simple (if rather hacky way) to prevent damage
to the Pi is to put 1K resistors on each of the programming lines.

The Pi GPIO pins are described [here](http://elinux.org/RPi_Low-level_peripherals#General_Purpose_Input.2FOutput_.28GPIO.29)
a good pinout of the Arduino programming header (the 3x2 connector) is given [here](http://forum.arduino.cc/index.php?topic=84190.0).
**Note** On the Dagu Mini Driver board, the pin names of the ISP header are given on the back of the
board.

The connections to make are (remember the level shifting!)

    GPIO 25  <---->   RESET (RST)
    GPIO 11  <---->   SCK
    GPIO 9   <---->   MISO (MI)
    GPIO 10  <---->   MOSI (MO)

![Wiring for the Raspberry Pi and Mini Driver](https://bitbucket.org/DawnRobotics/pi_isp/raw/master/images/pi_isp_wiring.png))
    
Flashing the Bootloader
=======================

If you've come here to reflash your Mini Driver's bootloader, you can do this by running

    sudo ./rescue_mini_driver
    
If you'd like to flash another bootloader you would run

    sudo ./flash_arduino_bootloader BOARD_TYPE
    
Where BOARD_TYPE is the Arduino to flash (i.e. uno). For a list of board types run

    ./flash_arduino_bootloader -l