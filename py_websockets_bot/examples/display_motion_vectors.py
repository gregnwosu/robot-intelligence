#! /usr/bin/python

# This example shows how motion vectors can be streamed from the robot's camera.

import time
import argparse
import math
import cv2
import numpy as np
import py_websockets_bot

MACRO_BLOCK_SIDE_LENGTH = 16

latest_flow_image = None

#---------------------------------------------------------------------------------------------------
def motion_vectors_callback( motion_vectors, motion_vectors_time ):
    
    global latest_flow_image
    
    delay = time.time() - motion_vectors_time
    print "Delay of {0} seconds from receiving data".format( delay )
    if delay > 0.5:
        print "Delay too large - dropping motion vectors"
        return
    
    start_time = time.time()
    
    # Convert the vectors to an image showing flow
    image_width = motion_vectors.shape[ 1 ] * MACRO_BLOCK_SIDE_LENGTH
    image_height = motion_vectors.shape[ 0 ] * MACRO_BLOCK_SIDE_LENGTH
    flow_image = np.zeros( ( image_height, image_width ), dtype=np.uint8 )
    
    for block_y in range( motion_vectors.shape[ 0 ] ):
        
        start_y = block_y*MACRO_BLOCK_SIDE_LENGTH + MACRO_BLOCK_SIDE_LENGTH/2
        
        for block_x in range( motion_vectors.shape[ 1 ] ):
            
            start_x = block_x*MACRO_BLOCK_SIDE_LENGTH + MACRO_BLOCK_SIDE_LENGTH/2
            
            end_x = start_x - motion_vectors[ "x" ][ block_y, block_x ]
            end_y = start_y - motion_vectors[ "y" ][ block_y, block_x ]
            
            length = math.sqrt( (end_x - start_x)**2 + (end_y - start_y)**2 )
            
            #if length >= 0 and length < 40 \
            #    and motion_vectors[ "sad" ][ block_y, block_x ] < 300:
                
            cv2.line( flow_image, ( start_x, start_y ), ( end_x, end_y ),
                ( 255, 255, 255 ), 2 )
    
    latest_flow_image = flow_image
    
    end_time = time.time()
    print "Processing took {0} seconds".format( end_time - start_time )
    
#---------------------------------------------------------------------------------------------------        
if __name__ == "__main__":

    # Set up a parser for command line arguments
    parser = argparse.ArgumentParser( "Gets motion vectors from the robot and displays them" )
    parser.add_argument( "hostname", default="localhost", nargs='?', help="The ip address of the robot" )

    args = parser.parse_args()
 
    # Connect to the robot
    bot = py_websockets_bot.WebsocketsBot( args.hostname )

    # Start streaming motion vectors from the camera
    bot.start_streaming_motion_vectors( motion_vectors_callback )

    # Run in a loop until the user presses Ctrl+C to quit
    try:
    
        while True:
            bot.update()
            
            if latest_flow_image != None:
                cv2.imshow( "flow", latest_flow_image )
                
            cv2.waitKey( 1 )
                
    except KeyboardInterrupt:
        pass    # Catch Ctrl+C
        
    # Disconnect from the robot
    bot.disconnect()

    

