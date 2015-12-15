#! /usr/bin/python

# This example shows the simplest way of getting an image from the robot's camera. The image
# is an OpenCV image so we also show how to perform edge detection on the image

import time
import argparse
import cv2
import py_websockets_bot

#---------------------------------------------------------------------------------------------------        
if __name__ == "__main__":

    # Set up a parser for command line arguments
    parser = argparse.ArgumentParser( "Gets an image from the robot" )
    parser.add_argument( "hostname", default="localhost", nargs='?', help="The ip address of the robot" )

    args = parser.parse_args()
 
    # Connect to the robot
    bot = py_websockets_bot.WebsocketsBot( args.hostname )

    # Start streaming images from the camera
    bot.start_streaming_camera_images()

    # Get an image from the robot
    image, image_time = bot.get_latest_camera_image()

    # Display the image
    cv2.imshow( "Image", image )
 
    # Convert to grayscale
    gray_image = cv2.cvtColor( image, cv2.COLOR_RGB2GRAY )

    # Perform edge detection on the image
    edge_image = cv2.Canny( gray_image, threshold1=64, threshold2=192 )

    # Display the edge image
    cv2.imshow( "Edge Image", edge_image )

    # Wait for the user to press a key
    cv2.waitKey( 0 )

    # Disconnect from the robot
    bot.disconnect()
