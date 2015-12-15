#! /usr/bin/python

# This example shows how to use the websockets interface to make the robot move around

import time
import argparse
import py_websockets_bot

#---------------------------------------------------------------------------------------------------
if __name__ == "__main__":

    # Set up a parser for command line arguments
    parser = argparse.ArgumentParser( "Moves the robot around" )
    parser.add_argument( "hostname", default="localhost", nargs='?', help="The ip address of the robot" )

    args = parser.parse_args()

    # Connect to the robot
    bot = py_websockets_bot.WebsocketsBot( args.hostname )

    # Drive forwards
    bot.set_motor_speeds( 80.0, 80.0 )
    time.sleep( 2.5 )

    # Stop
    bot.set_motor_speeds( 0.0, 0.0 )
    time.sleep( 0.75 )

    # Turn left
    bot.set_motor_speeds( -60.0, 60.0 )
    time.sleep( 0.25 )

    # Stop
    bot.set_motor_speeds( 0.0, 0.0 )
    time.sleep( 0.75 )

    # Look left
    bot.set_neck_angles( pan_angle_degrees=0.0, tilt_angle_degrees=90.0 )
    time.sleep( 1.0 )

    # Look right
    bot.set_neck_angles( pan_angle_degrees=180.0, tilt_angle_degrees=90.0 )
    time.sleep( 1.0 )

    # Centre the neck
    bot.centre_neck()
    time.sleep( 1.0 )

    # Drive forwards
    bot.set_motor_speeds( 80.0, 80.0 )
    time.sleep( 0.5 )

    # Stop
    bot.set_motor_speeds( 0.0, 0.0 )
    time.sleep( 0.75 )

    # Spin right
    bot.set_motor_speeds( 80.0, -80.0 )
    time.sleep( 2.5 )

    # Stop
    bot.set_motor_speeds( 0.0, 0.0 )
    time.sleep( 0.75 )

    # Disconnect from the robot
    bot.disconnect()
