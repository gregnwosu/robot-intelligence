#! /usr/bin/python

# As an alternative to getting images using the blocking get_latest_camera_image routine, you
# can also get images using a callback.

import time
import argparse
import cv2
import py_websockets_bot

latest_camera_image = None
latest_small_camera_image = None

#---------------------------------------------------------------------------------------------------
def camera_image_callback( image, image_time ):
    
    global latest_camera_image
    
    # Put image processing here...
    
    latest_camera_image = image
    
#---------------------------------------------------------------------------------------------------
def camera_small_image_callback( image, image_time ):
    
    global latest_small_camera_image
    
    # Put image processing here...
    
    latest_small_camera_image = image
    
#---------------------------------------------------------------------------------------------------        
if __name__ == "__main__":

    # Set up a parser for command line arguments
    parser = argparse.ArgumentParser( "Gets images from the robot using callbacks and displays them" )
    parser.add_argument( "hostname", default="localhost", nargs='?', help="The ip address of the robot" )

    args = parser.parse_args()
 
    # Connect to the robot
    bot = py_websockets_bot.WebsocketsBot( args.hostname )

    # Start streaming images from the camera
    bot.start_streaming_camera_images( camera_image_callback )
    bot.start_streaming_small_camera_images( camera_small_image_callback )

    # Run in a loop until the user presses Ctrl+C to quit
    try:
    
        while True:
            bot.update()
            
            if latest_camera_image != None:
                cv2.imshow( "image", latest_camera_image )
                
            if latest_small_camera_image != None:
                cv2.imshow( "small_image", latest_small_camera_image )
                
            cv2.waitKey( 1 )
                
    except KeyboardInterrupt:
        pass    # Catch Ctrl+C
        
    # Disconnect from the robot
    bot.disconnect()

    

