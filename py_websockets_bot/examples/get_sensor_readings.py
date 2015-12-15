#! /usr/bin/python

# This example shows how to configure, and read from, the robot's sensors

import time
import argparse
import py_websockets_bot
import py_websockets_bot.mini_driver
import py_websockets_bot.robot_config
   
#---------------------------------------------------------------------------------------------------        
if __name__ == "__main__":

    # Set up a parser for command line arguments
    parser = argparse.ArgumentParser( "Gets sensor readings from the robot and displays them" )
    parser.add_argument( "hostname", default="localhost", nargs='?', help="The ip address of the robot" )

    args = parser.parse_args()
 
    # Connect to the robot
    bot = py_websockets_bot.WebsocketsBot( args.hostname )

    # Estimate the offset between our system clock, and the robot's system clock. You only need
    # to do this if you're interested in the age of the sensor readings. If you're not interested
    # then you can leave it out of your own programs
    print "Estimating robot time offset (this will take about 10 seconds)..."
    robot_time_offset = bot.estimate_robot_time_offset()
    
    # Configure the sensors on the robot
    sensorConfiguration = py_websockets_bot.mini_driver.SensorConfiguration(
        configD12=py_websockets_bot.mini_driver.PIN_FUNC_ULTRASONIC_READ, 
        configD13=py_websockets_bot.mini_driver.PIN_FUNC_DIGITAL_READ, 
        configA0=py_websockets_bot.mini_driver.PIN_FUNC_ANALOG_READ, 
        configA1=py_websockets_bot.mini_driver.PIN_FUNC_ANALOG_READ,
        configA2=py_websockets_bot.mini_driver.PIN_FUNC_ANALOG_READ, 
        configA3=py_websockets_bot.mini_driver.PIN_FUNC_DIGITAL_READ,
        configA4=py_websockets_bot.mini_driver.PIN_FUNC_ANALOG_READ, 
        configA5=py_websockets_bot.mini_driver.PIN_FUNC_ANALOG_READ,
        leftEncoderType=py_websockets_bot.mini_driver.ENCODER_TYPE_QUADRATURE, 
        rightEncoderType=py_websockets_bot.mini_driver.ENCODER_TYPE_QUADRATURE )
    
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
                
                # Uncomment this to view the current robot configuration
                #robot_config = bot.get_robot_config()
                #print robot_config
                #print ""
                
                print "robot time offset :", robot_time_offset
                print ""
                
                # Print out each sensor reading in turn
                sensor_dict = status_dict[ "sensors" ]
                for sensor_name in sensor_dict:
                    
                    # Get the timestamp and data for the reading
                    timestamp = sensor_dict[ sensor_name ][ "timestamp" ]
                    data = sensor_dict[ sensor_name ][ "data" ]
                    
                    # Calculate the age of the reading
                    reading_age = (time.time() + robot_time_offset) - timestamp
                    
                    # Print out information about the reading
                    print sensor_name, ":", data, "reading age :", reading_age
                    
            print ""
            
            time.sleep( 0.05 )
                
    except KeyboardInterrupt:
        pass    # Catch Ctrl+C
        
    # Disconnect from the robot
    bot.disconnect()

    

