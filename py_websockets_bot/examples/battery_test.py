#! /usr/bin/python

# This example uses the websockets interface to make the robot move around whilst
# periodically reading the battery voltage. If the robot is put in a safe place
# then this script can be used to get an idea of how long your batteries will last.

import time
import argparse
import py_websockets_bot
import random
import csv

IDLE = "Idle"
TURNING = "Turning"

TIME_BETWEEN_BATTERY_READS = 1.0

TIME_BETWEEN_TURNS = 10.0
TURNING_TIME = 3.0
TIME_BETWEEN_LOOKS = 1.0

MIN_PAN_ANGLE = 90.0 - 60.0
MAX_PAN_ANGLE = 90.0 + 60.0

#---------------------------------------------------------------------------------------------------
if __name__ == "__main__":

    # Set up a parser for command line arguments
    parser = argparse.ArgumentParser( "Script for testing robot battery life" )
    parser.add_argument( "hostname", default="localhost", nargs='?', help="The ip address of the robot" )

    args = parser.parse_args()
    
    # Connect to the robot
    bot = py_websockets_bot.WebsocketsBot( args.hostname )

    robot_state = IDLE
    state_start_time = time.time()
    last_look_time = time.time()
    
    last_battery_read_time = time.time()
    battery_readings = []
    
    start_time = time.time()
    
    try:
        while True:
            
            # Update the robot's state machine
            if robot_state == IDLE:
                
                bot.set_motor_speeds( 0.0, 0.0 )
                
                if time.time() - last_look_time > TIME_BETWEEN_LOOKS:
                    last_look_time = time.time()
                    pan_angle = MIN_PAN_ANGLE + random.random()*(MAX_PAN_ANGLE - MIN_PAN_ANGLE)
                    bot.set_neck_angles( pan_angle, 70.0 )
                
                if time.time() - state_start_time > TIME_BETWEEN_TURNS:
                    robot_state = TURNING
                    state_start_time = time.time()
                    last_look_time = time.time()
                    turning_left = random.random() < 0.5
                        
            elif robot_state == TURNING:
                
                if turning_left:
                    bot.set_motor_speeds( -60.0, 60.0 )
                else:
                    bot.set_motor_speeds( 60.0, -60.0 )
                
                if time.time() - state_start_time > TURNING_TIME:
                    robot_state = IDLE
                    state_start_time = time.time()
                    last_look_time = time.time()

            # Check to see if we need to read battery voltage
            if time.time() - last_battery_read_time > TIME_BETWEEN_BATTERY_READS:
                
                voltage, read_time = bot.get_battery_voltage()
                battery_readings.append( { "time" : read_time - start_time, "voltage" : voltage } )
                last_battery_read_time = time.time()
            
                print "Voltage is", voltage, "at", battery_readings[ -1 ][ "time" ]
                    
            # Update any background communications with the robot
            bot.update()
            
            # Sleep for a bit so that we don't overload the web server on the robot
            time.sleep( 0.05 )
            
    except KeyboardInterrupt:
        pass    # Catch Ctrl+C
    except Exception as e:
        
        print "Exception caught, exiting program after {0} seconds...".format( time.time() - start_time )
        print e
   
    # Write out battery voltage readings to a file
    with open( "battery.csv", "w" ) as csv_file:
        dict_writer = csv.DictWriter( csv_file, [ "time", "voltage" ] )
        dict_writer.writeheader()
        dict_writer.writerows( battery_readings )
   
    # Disconnect from the robot
    bot.set_motor_speeds( 0.0, -0.0 )
    bot.disconnect()

    