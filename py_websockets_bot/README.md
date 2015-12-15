This library uses websockets to talk to the [raspberry_pi_camera_bot](https://bitbucket.org/DawnRobotics/raspberry_pi_camera_bot)
web server. This allows you to programmatically control the robot either from a script running locally on the robot's Raspberry Pi,
or from a script that runs on a separate computer, connected to the robot by WiFi.

Dependencies
------------

This library requires OpenCV and numpy to retrieve and decode camera images which come back from the Pi 

Installing on the Raspberry Pi or Another Linux Machine
-------------------------------------------------------

Run the following commands to install the dependencies

    sudo apt-get update
    sudo apt-get install python-opencv

then

    git clone https://bitbucket.org/DawnRobotics/py_websockets_bot.git
    cd py_websockets_bot
    sudo python setup.py install

Installing on Windows
---------------------

This is trickier but involves the following steps

* Install Python 2.7.3 - https://www.python.org/download/releases/2.7.3
* Install Numpy 1.6.2 
    - Goto http://sourceforge.net/projects/numpy/files/NumPy/1.6.2/
    - Download and install numpy-1.6.2-win32-superpack-python2.7.exe
* Download OpenCV 2.3.1 - http://opencv.org/downloads.html - 2.3.1 Windows superpack
* Unpack OpenCV to any folder
* Goto opencv/build/python/2.7 folder
* Copy cv2.pyd to C:/Python27/lib/site-packeges.
    
If needed, more details for OpenCV setup on Windows can be found at http://docs.opencv.org/trunk/doc/py_tutorials/py_setup/py_setup_in_windows/py_setup_in_windows.html

* Now download this library (repository) from https://bitbucket.org/DawnRobotics/py_websockets_bot/downloads
* Unpack the library to a folder and navigate to it on the command line
* Run the following command from the command line

    c:\Python27\python.exe setup.py install

Installing on Mac
-----------------

No idea, but should hopefully be similar to installing on Linux

Running
-------

You can find examples for using the interface in the examples folder. When you run the examples you can pass 
in the IP address of the robot (this defaults to localhost).

So run

    examples/motor_test.py ROBOT_IP_ADDRESS     
    
to make the robot move, and run

    examples/get_image.py ROBOT_IP_ADDRESS
    
to see how you can get images from the Raspberry Pi's camera


