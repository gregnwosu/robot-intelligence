#! /usr/bin/python

# This example shows how to configure, and read from, the line follower sensors attached to
# the mini driver. For this sample it is assumed that you have 3 line follower sensors attached
# to pins A1, A2 and A5 as shown in the 'Digital Inputs' section of the tutorial at
# http://blog.dawnrobotics.co.uk/2014/09/adding-sensors-raspberry-pi-camera-robot-kit/

import time
import argparse
import py_websockets_bot
import py_websockets_bot.mini_driver
import py_websockets_bot.robot_config

#---------------------------------------------------------------------------------------------------        
def is_bit_set( number, bit_idx ):
    
    """This helper routine returns True if a given bit is set in a given number. Bits are numbered
       from 0 and are ordered right to left. So a byte (which consists of 8 bits) would have its
       bits numbered as follows (7 6 5 4 3 2 1 0)"""
       
    return (number & (1 << bit_idx)) != 0

#---------------------------------------------------------------------------------------------------        
if __name__ == "__main__":

    # Set up a parser for command line arguments
    parser = argparse.ArgumentParser( "Gets line follower sensor readings from the robot and displays them" )
    parser.add_argument( "hostname", default="localhost", nargs='?', help="The ip address of the robot" )

    args = parser.parse_args()
 
    # Connect to the robot
    bot = py_websockets_bot.WebsocketsBot( args.hostname )
    
    # Configure the sensors on the robot. The pins that the line follower sensors are
    # set to be digital inputs. No other pins are specified so they will adobt their default values
    sensorConfiguration = py_websockets_bot.mini_driver.SensorConfiguration(
        configA1=py_websockets_bot.mini_driver.PIN_FUNC_DIGITAL_READ, 
        configA2=py_websockets_bot.mini_driver.PIN_FUNC_DIGITAL_READ,
        configA5=py_websockets_bot.mini_driver.PIN_FUNC_DIGITAL_READ )
    
    # We set the sensor configuration by getting the current robot configuration and modifying it.
    # In this way we don't trample on any other configuration settings
    robot_config = bot.get_robot_config()
    robot_config.miniDriverSensorConfiguration = sensorConfiguration
    
    bot.set_robot_config( robot_config )
    
    # Run in a loop until the user presses Ctrl+C to quit
    try:
    
        while True:
            bot.update()
            
            # The sensor readings are returned as part of the status of the robot
            status_dict, read_time = bot.get_robot_status_dict()
            
            if not "sensors" in status_dict:
                
                print "No sensor readings..."
                print status_dict
                
            else:
                
                # Get the sensor reading for the digital inputs
                sensor_dict = status_dict[ "sensors" ]
                digital_data = sensor_dict[ "digital" ][ "data" ]
                
                # The digital sensor readings are returned as a byte. From left to right
                # the bits of the byte correspond to the pins A5, A4, A3, A2, A1, A0, D13 and D12
                #                                              7   6   5   4   3   2    1       0
                reading_A1 = is_bit_set( digital_data, 3 )
                reading_A2 = is_bit_set( digital_data, 4 )
                reading_A5 = is_bit_set( digital_data, 7 )
                
                # Now, the line sensors we sell are slightly odd in that they return a high value
                # if no line is detected, and a low value if a line is detected (you should also
                # see an LED turn on on the back of the sensor). We therefore need to invert the
                # reading
                line_detected_A1 = not reading_A1
                line_detected_A2 = not reading_A2
                line_detected_A5 = not reading_A5
                
                # Print out the sensor readings.
                print "line_detected_A1 :", line_detected_A1
                print "line_detected_A2 :", line_detected_A2
                print "line_detected_A5 :", line_detected_A5
                    
            print ""
            
            time.sleep( 0.05 )
                
    except KeyboardInterrupt:
        pass    # Catch Ctrl+C
        
    # Disconnect from the robot
    bot.disconnect()

    

