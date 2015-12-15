# Copyright (c) 2014, Dawn Robotics Ltd
# All rights reserved.

# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, 
# this list of conditions and the following disclaimer in the documentation 
# and/or other materials provided with the distribution.

# 3. Neither the name of the Dawn Robotics Ltd nor the names of its contributors 
# may be used to endorse or promote products derived from this software without 
# specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
This package provides the :class:`WebsocketsBot` class which uses websockets to communicate with
and control a robot running the `raspberry_pi_camera_bot <https://bitbucket.org/DawnRobotics/raspberry_pi_camera_bot/>`_
robot_web_server.py


WebsocketsBot
=============

    .. autoclass:: WebsocketsBot()
        :special-members:
        :members:

CameraStreamingData
===================

    .. autoclass:: CameraStreamingData()
        :members:
        
ImageReaderProcess
==================

    .. autoclass:: ImageReaderProcess()
        :members:
        
MotionVectorsReaderProcess
==========================

    .. autoclass:: MotionVectorsReaderProcess()
        :members:
"""

import math
import time
import json
import httplib
import websocket
import multiprocessing

import robot_config
import mini_driver

try:
    import numpy as np
    import cv2
except ImportError:
    print "Error: Both numpy and OpenCV (cv2) are required"
    numpy = object()    # Mock up for docs build
    cv2 = object()

#---------------------------------------------------------------------------------------------------
class ImageReaderProcess( multiprocessing.Process ):
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, hostname, camera_port, image_queue, stream_small_images=False, max_update_rate_hz=60.0 ):
        
        multiprocessing.Process.__init__( self )
        
        self._hostname = hostname
        self._camera_port = camera_port
        self._image_queue = image_queue
        self._stream_small_images = stream_small_images
        
        self._max_update_rate_hz = max_update_rate_hz
        if self._max_update_rate_hz <= 0.0:
            self._max_update_rate_hz = 1.0
        
        self._exit_event = multiprocessing.Event()

    #-----------------------------------------------------------------------------------------------
    def shutdown( self ):
        
        self._exit_event.set()
        
    #-----------------------------------------------------------------------------------------------
    def run( self ):
        
        # Run in a loop continually getting images from the server
        # TODO: Convert to using action=streaming
        while not self._exit_event.is_set():
            
            start_time = time.time()
            
            image_data = None
        
            try:
                # Make the request to the web server
                connection = httplib.HTTPConnection( self._hostname, self._camera_port, timeout=3.0 )
                
                if self._stream_small_images:
                    connection.request( "GET", "/small_image/?action=snapshot" )
                else:
                    connection.request( "GET", "/?action=snapshot" )
                    
                response = connection.getresponse()
                
                if response.status == httplib.OK:
                    
                    image_data = response.read()
                    
            except:
                pass    # Ignore exceptions which arise from connection errors or invalid data

            if image_data != None:
                if not self._image_queue.full():
                    self._image_queue.put_nowait( ( image_data, time.time() ) )
                else:
                    pass
                    #if self._stream_small_images:
                        #print "Dropped small image frame"
                    #else:
                        #print "Dropped image frame"
                    
            end_time = time.time()
            sleep_ime = 1.0/self._max_update_rate_hz - (end_time - start_time)
            if sleep_ime > 0.0:
                time.sleep( sleep_ime )
                    
        self._image_queue.cancel_join_thread()

#---------------------------------------------------------------------------------------------------
class MotionVectorsReaderProcess( multiprocessing.Process ):
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, hostname, camera_port, motion_vectors_queue, max_update_rate_hz=60.0 ):
        
        multiprocessing.Process.__init__( self )
        
        self._hostname = hostname
        self._camera_port = camera_port
        self._motion_vectors_queue = motion_vectors_queue
        
        self._max_update_rate_hz = max_update_rate_hz
        if self._max_update_rate_hz <= 0.0:
            self._max_update_rate_hz = 1.0
            
        self._exit_event = multiprocessing.Event()

    #-----------------------------------------------------------------------------------------------
    def shutdown( self ):
        
        self._exit_event.set()
        
    #-----------------------------------------------------------------------------------------------
    def run( self ):
        
        # Run in a loop continually getting motion vectors from the server
        # TODO: Convert to using action=streaming
        while not self._exit_event.is_set():
            
            start_time = time.time()
            
            motion_vectors_data = None
        
            try:
                # Make the request to the web server
                connection = httplib.HTTPConnection( self._hostname, self._camera_port )
                connection.request( "GET", "/motion_vectors/?action=snapshot" )
                response = connection.getresponse()
                
                if response.status == httplib.OK:
                    
                    motion_vectors_data = response.read()
                    
            except:
                pass    # Ignore exceptions which arise from connection errors or invalid data
            
            if motion_vectors_data != None:
                if not self._motion_vectors_queue.full():

                    self._motion_vectors_queue.put_nowait( ( motion_vectors_data, time.time() ) )
                
                else:
                    pass
                    #print "Dropped motion vectors frame"
                
            end_time = time.time()
            sleep_ime = 1.0/self._max_update_rate_hz - (end_time - start_time)
            if sleep_ime > 0.0:
                time.sleep( sleep_ime )
            
        self._motion_vectors_queue.cancel_join_thread()
        
#---------------------------------------------------------------------------------------------------
class CameraStreamingData:
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, data_processing_routine ):
        
        self.reader_process = None
        self.callback = None
        self.data_queue = multiprocessing.Queue( 5 )
        self.last_processed_data = None
        self.last_processed_data_time = 0.0
        self.data_processing_routine = data_processing_routine
        
#---------------------------------------------------------------------------------------------------
class WebsocketsBot:
    
    """Interface class for talking to a robot running the `raspberry_pi_camera_bot <https://bitbucket.org/DawnRobotics/raspberry_pi_camera_bot/>`_
       robot_web_server.py using websockets"""
    
    TIME_BETWEEN_CAMERA_KEEP_ALIVES = 0.25
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, hostname="localhost", port=80, camera_port=8080 ):
        
        """Constructs a :py:class:`WebsocketsBot`
        
           :param str hostname: The ip address of the robot
           :param int port: The port of the robot's web server
           :param int camera_port: The port of the web server providing camera images"""

        self._hostname = hostname
        self._port = port
        self._camera_port = camera_port
        
        url = "ws://{0}:{1}/robot_control/websocket".format( hostname, port )
        print "Connecting to", url
        self._websocket = websocket.create_connection( url )
        
        # These placeholders keep track of the process of streaming different types of data from 
        # the robot's camera
        self._streaming_data = {
            "image" : CameraStreamingData( self._process_raw_image_data ),
            "small_image" : CameraStreamingData( self._process_raw_image_data ),
            "motion_vector" : CameraStreamingData( self._process_raw_motion_vectors_data )
        }
        
        self._last_camera_keep_alive_time = 0.0
    
    #-----------------------------------------------------------------------------------------------
    def disconnect( self ):
        
        """This routine should be called when the :py:class:`WebsocketsBot` is no longer needed"""
        
        for data_name in self._streaming_data:
            self._stop_streaming( data_name )
    
    #-----------------------------------------------------------------------------------------------
    def start_streaming_camera_images( self, image_callback=None ):
        
        """This routine must be called to start streaming images from the robot's camera. Images
           can either be obtained in a blocking fashion, by calling :py:func:`get_latest_camera_image`,
           or an image callback can be provided.
           
           :param func image_callback: A function to call with an image and timestamp when \
           an image is obtained."""
        
        def create_streaming_process( data_queue ):
            return ImageReaderProcess( self._hostname, self._camera_port, data_queue )
        
        self._start_streaming( "image", create_streaming_process, image_callback )
        
    #-----------------------------------------------------------------------------------------------
    def stop_streaming_camera_images( self ):
        
        """Stops streaming images from the robot's camera"""
        
        self._stop_streaming( "image" )
    
    #-----------------------------------------------------------------------------------------------
    def get_latest_camera_image( self, max_image_age=None ):
        
        """This routine will block until an image is obtained from the robot's camera.
           
           :param float max_image_age: Specifies the maximum age of the image to retrieve in seconds. \
           If this is None, then the routine will retrieve the first available image. If the currently \
           available image is older than this age then the routine blocks until a newer image is retrieved.
           
           :return: a tuple containing the image and its timestamp
           
           :raises Exception: if :py:func:`start_streaming_camera_images` has not been called"""
        
        return self._get_latest_streaming_data( "image", "start_streaming_camera_images", max_image_age )
                    
    #-----------------------------------------------------------------------------------------------
    def start_streaming_small_camera_images( self, image_callback=None ):
        
        """This routine must be called to start streaming 'small' images from the robot's camera. Images
           can either be obtained in a blocking fashion, by calling :py:func:`get_latest_small_camera_image`,
           or an image callback can be provided.
           
           A small image is the normal camera image, reduced to a size of 160x120. Processing smaller images
           can give a large speed up to a lot of computer vision algorithms, and this is very important when
           running on a computationally constrained platform such as the Raspberry Pi.
           
           :param func image_callback: A function to call with an image and timestamp when \
           an image is obtained."""
        
        def create_streaming_process( data_queue ):
            return ImageReaderProcess( self._hostname, self._camera_port, data_queue, stream_small_images=True )
        
        self._start_streaming( "small_image", create_streaming_process, image_callback )
        
    #-----------------------------------------------------------------------------------------------
    def stop_streaming_small_camera_images( self ):
        
        """Stops streaming 'small' images from the robot's camera"""
        
        self._stop_streaming( "small_image" )
    
    #-----------------------------------------------------------------------------------------------
    def get_latest_small_camera_image( self, max_image_age=None ):
        
        """This routine will block until an image is obtained from the robot's camera.
           
           :param float max_image_age: Specifies the maximum age of the image to retrieve in seconds. \
           If this is None, then the routine will retrieve the first available image. If the currently \
           available image is older than this age then the routine blocks until a newer image is retrieved.
           
           :return: a tuple containing the image and its timestamp
           
           :raises Exception: if :py:func:`start_streaming_small_camera_images` has not been called"""
        
        return self._get_latest_streaming_data( "small_image", "start_streaming_small_camera_images", max_image_age )
    
    #-----------------------------------------------------------------------------------------------
    def start_streaming_motion_vectors( self, motion_vectors_callback=None ):
        
        """This routine must be called to start streaming motion vectors from the robot's camera.
           Motion vectors can either be obtained in a blocking fashion, by calling :py:func:`get_latest_motion_vectors`,
           or a callback can be provided.
           
           :param func motion_vectors_callback: A function to call with motion vectors and a timestamp when \
           a new batch of motion vectors is obtained."""
        
        def create_streaming_process( data_queue ):
            return MotionVectorsReaderProcess( self._hostname, self._camera_port, data_queue )
        
        self._start_streaming( "motion_vector", create_streaming_process, motion_vectors_callback )
        
    #-----------------------------------------------------------------------------------------------
    def stop_streaming_motion_vectors( self ):
        
        """Stops streaming motion vectors from the robot's camera"""
        
        self._stop_streaming( "motion_vector" )
    
    #-----------------------------------------------------------------------------------------------
    def get_latest_motion_vectors( self, max_motion_vectors_age=None ):
        
        """This routine will block until an image is obtained from the robot's camera.
           
           :param float max_motion_vectors_age: Specifies the maximum age of the motion vectors to retrieve \
           in seconds. If this is None, then the routine will retrieve the first available set of motion \
           vectors. If the currently available motion vectors are older than this age then the routine blocks \
           until a newer set of motion vectors is retrieved.
           
           :return: a tuple containing the motion vectors and a timestamp
           
           :raises Exception: if :py:func:`stop_streaming_motion_vectors` has not been called"""
        
        return self._get_latest_streaming_data( "motion_vector", "stop_streaming_motion_vectors", max_motion_vectors_age )
    
    #-----------------------------------------------------------------------------------------------
    def _start_streaming( self, data_name, create_streaming_process_func, callback=None ):
        
        streaming_data = self._streaming_data[ data_name ]
        
        if streaming_data.reader_process == None:
            
            streaming_data.reader_process = create_streaming_process_func( streaming_data.data_queue )
            streaming_data.reader_process.start()
        
        streaming_data.callback = callback
            
        self._update_camera_keep_alive()
    
    #-----------------------------------------------------------------------------------------------
    def _stop_streaming( self, data_name ):
        
        streaming_data = self._streaming_data[ data_name ]
        
        if streaming_data.reader_process != None:
            
            # Start the shutdown
            streaming_data.reader_process.shutdown()

            # Clear out the data queue
            while not streaming_data.data_queue.empty():
                streaming_data.data_queue.get_nowait()
            
            # Wait for the process to finish
            streaming_data.reader_process.join()
            
            # Clear out the process and callback
            streaming_data.reader_process = None
            streaming_data.callback = None
                        
            # Double  check that the queue is clear
            while not streaming_data.data_queue.empty():
                streaming_data.data_queue.get_nowait()
    
    #-----------------------------------------------------------------------------------------------
    def _get_latest_streaming_data( self, data_name, start_streaming_routine_name, max_data_age=None ):
    
        streaming_data = self._streaming_data[ data_name ]
    
        if streaming_data.reader_process == None:
            raise Exception( "Trying to get data before {0} has been called".format( start_streaming_routine_name ) )
        
        processed_data = streaming_data.last_processed_data
        processed_data_time = streaming_data.last_processed_data_time
        
        while processed_data == None \
            or ( max_data_age != None and time.time() - processed_data_time > max_data_age ):
            
            time.sleep( 0.001 )
            self.update()
            
            processed_data = streaming_data.last_processed_data
            processed_data_time = streaming_data.last_processed_data_time
        
        return processed_data, processed_data_time
    
    #-----------------------------------------------------------------------------------------------
    def _update_camera_keep_alive( self ):
        
        # If we're trying to read camera images or motion vectors then we need
        # to periodically poke the server
        streaming_data_from_camera = False
        
        for data_name in self._streaming_data:
            
            if self._streaming_data[ data_name ].reader_process != None:
                
                streaming_data_from_camera = True
                break
        
        if streaming_data_from_camera:
            
            if time.time() - self._last_camera_keep_alive_time >= self.TIME_BETWEEN_CAMERA_KEEP_ALIVES:
                
                self._websocket.send( "StartStreaming" )
                self._last_camera_keep_alive_time = time.time()
    
    #-----------------------------------------------------------------------------------------------
    def set_motor_speeds( self, left_motor_speed, right_motor_speed ):
        
        """Sets the motor speeds of the robot
           
           :param float left_motor_speed: Left motor speed from 0 to 100%
           :param float right_motor_speed: Right motor speed from 0 to 100%"""
        
        self._websocket.send( "SetMotorSpeeds {0} {1}".format( left_motor_speed, right_motor_speed ) )

    #-----------------------------------------------------------------------------------------------
    def set_neck_angles( self, pan_angle_degrees, tilt_angle_degrees ):
        
        """Sets the neck angles of the robot in degrees. The centre point for each motor is 90 degrees
           
           :param float pan_angle_degrees: Pan servo angle from 0 to 180 degrees
           :param float tilt_angle_degrees: Tilt servo angle from 0 to 180 degrees"""
        
        self._websocket.send( "SetNeckAngles {0} {1}".format( pan_angle_degrees, tilt_angle_degrees ) )
    
    #-----------------------------------------------------------------------------------------------
    def set_motor_joystick_pos( self, joystickX, joystickY ):
        
        """Passes a movement joystick command to the robot. The magnitude of the joystick
           input should be less than or equal to 1.0. So
           
           :math:`\sqrt{ \mathrm{joystickX}^2 + \mathrm{joystickY}^2 } \le 1.0,`
           
           however, if it's not then it will be constrained automatically.
           
           :param float joystickX: Joystick X input from -1.0 (left) to 1.0 (right)
           :param float joystickY: Joystick Y input from -1.0 (backwards) to 1.0 (forwards)"""
        
        self._websocket.send( "Move {0} {1}".format( joystickX, joystickY ) )
        
    #-----------------------------------------------------------------------------------------------
    def set_neck_joystick_pos( self, joystickX, joystickY ):
        
        """Passes a neck joystick command to the robot. The magnitude of the joystick
           input should be less than or equal to 1.0. So
           
           :math:`\sqrt{ \mathrm{joystickX}^2 + \mathrm{joystickY}^2 } \le 1.0,`
           
           however, if it's not then it will be constrained automatically.
           
           :param float joystickX: Joystick X input from -1.0 (left) to 1.0 (right)
           :param float joystickY: Joystick Y input from -1.0 (down) to 1.0 (up)"""
        
        self._websocket.send( "PanTilt {0} {1}".format( joystickX, joystickY ) )
    
    #-----------------------------------------------------------------------------------------------
    def get_robot_status_dict( self ):
        
        """Gets a dictionary containing various status variables from the robot. If robot sensor
           data is returned then it will be returned as a dictionary of sensor readings in the
           entry called 'sensors'. Each sensor reading is a dictionary consisting of a 'data'
           entry and a 'timestamp' entry, where the timestamp is the system clock time of the robot
           in seconds when the reading was taken.
        
            :return: A dictionary containing status variables from the robot, along with the client 
                    time when the status dictionary was received.
                    
           :rtype: (dict, float)"""
        
        self._websocket.send( "GetRobotStatus" )
        status_dict = json.loads( self._websocket.recv() )
        read_time = time.time()
        
        return status_dict, read_time

    #-----------------------------------------------------------------------------------------------
    def get_battery_voltage( self ):
        
        """Reads the battery voltage from the robot.
        
           :return: The battery voltage read from the robot, along with the client time when the 
                    battery voltage was received.
                    
           :rtype: (float, float)"""
        
        voltage = 0.0
        
        status_dict, read_time = self.get_robot_status_dict()
        
        if "batteryVoltage" in status_dict:
            voltage = status_dict[ "batteryVoltage" ]
        else:
            read_time = None    # No valid reading made
        
        return voltage, read_time
    
    #-----------------------------------------------------------------------------------------------
    def get_robot_config( self ):
        
        """Reads the current configuration of the robot.
        
           :return: A :py:class:`RobotConfig` object containing configuration parameters for the robot
                    
           :rtype: :py:class:`RobotConfig`"""
        
        self._websocket.send( "GetConfig" )
        config_dict = json.loads( self._websocket.recv() )
        
        config = robot_config.RobotConfig()
        config.readDataFromConfigDict( config_dict )
        
        return config
    
    #-----------------------------------------------------------------------------------------------
    def set_robot_config( self, config ):
        
        """Sends configuration parameters to the robot. In order to avoid disrupting existing 
           configuration settings it is recommended that you call :py:func:`get_robot_config` first,
           modify the configuration parameters you're interested in, and then call this routine
           with the modified robot configuration.
        
           :param :py:class:`RobotConfig` config: The configuration to send to the robot"""
        
        config_dict = config.getConfigDict()
        
        self._websocket.send( "SetConfig {0}".format( 
            json.dumps( config_dict, default=lambda o: o.__dict__, separators=(',',':') ) ) )
    
    #-----------------------------------------------------------------------------------------------
    def estimate_robot_time_offset( self ):
        
        """Estimates the time difference between our system clock and the robot's system clock. We
           use a simple algorithm based on the method given at http://www.mine-control.com/zack/timesync/timesync.html
           The estimation process will take about 10 to 20 seconds so should only be called occasionally
           (perhaps once every 30 minutes or so).
           
           TODO: Get this to work asynchronously...
        
           :return: The offset to add to the system time in order to get to the robot's time
                    
           :rtype: float"""
           
        NUM_SAMPLES = 6
        INTER_SAMPLE_WAIT_TIME = 1.5
        
        samples = []
        for i in range( NUM_SAMPLES ):
            
            # Record the current time on the client
            start_time = time.time()
            
            # Get the current time on the server
            self._websocket.send( "GetServerTime" )
            server_time = json.loads( self._websocket.recv() )[ "serverTime" ]
            
            # Estimate the latency as half the round trip time
            end_time = time.time()
            
            latency = (end_time - start_time)/2.0
            
            # Calculate the offset to get from client time to  server time
            offset = server_time - end_time + latency
            
            samples.append( ( latency, offset ) )
            
            if i < NUM_SAMPLES - 1:
                time.sleep( INTER_SAMPLE_WAIT_TIME )
                
        # Sort the samples on latency and extract the median latency
        sorted_samples = sorted( samples, key=lambda o: o[ 0 ] )
        median_latency = sorted_samples[ int( NUM_SAMPLES/2 ) ][ 0 ]
        
        # Calculate the standard deviation of the latency
        latency_sum = 0.0
        for i in range( NUM_SAMPLES ):
            latency_sum += sorted_samples[ i ][ 0 ]
            
        mean_latency = latency_sum/NUM_SAMPLES
        
        squared_diff_sum = 0.0
        for i in range( NUM_SAMPLES ):
            squared_diff_sum += (mean_latency - sorted_samples[ i ][ 0 ])**2
            
        latency_sd = math.sqrt( squared_diff_sum/NUM_SAMPLES )
        
        # Calculate the average offset from all samples within one standard deviation of the
        # median latency
        offset_sum = 0.0
        num_offset_samples = 0
        
        for i in range( NUM_SAMPLES ):
            
            if abs( median_latency - sorted_samples[ i ][ 0 ] ) <= latency_sd:
                offset_sum += sorted_samples[ i ][ 1 ]
                num_offset_samples += 1
            
        return offset_sum/num_offset_samples
    
    #-----------------------------------------------------------------------------------------------
    def centre_neck( self ):
        
        """Centres the neck of the robot"""
        
        self._websocket.send( "Centre" )
        
    #-----------------------------------------------------------------------------------------------
    def power_off_robot( self ):
        
        """Instructs the Pi on the robot to shut down"""
        
        self._websocket.send( "Shutdown" )
    
    #-----------------------------------------------------------------------------------------------
    def update( self ):
        
        """This routine must be called periodically to ensure that background commications with the robot
           (to keep camera images streaming for example) happen on a regular basis."""

        # Keep camera alive if needed
        self._update_camera_keep_alive()
        
        # Update streaming data
        for data_name in self._streaming_data:
            
            streaming_data = self._streaming_data[ data_name ]
            
            num_items_to_process = streaming_data.data_queue.qsize()
            while num_items_to_process > 0 and not streaming_data.data_queue.empty():
                
                num_items_to_process -= 1
                
                raw_data, data_time = streaming_data.data_queue.get_nowait()
                
                if raw_data != None:
                    
                    processed_data = streaming_data.data_processing_routine( raw_data )                        
                    if processed_data != None:
                        
                        streaming_data.last_processed_data = processed_data
                        streaming_data.last_processed_data_time = data_time
                        
                        if streaming_data.callback != None:
                            
                            streaming_data.callback( processed_data, data_time )
                        
    #-----------------------------------------------------------------------------------------------
    def _process_raw_image_data( self, image_data ):
        
        image = None
        
        try:
            npArray = np.fromstring( image_data, np.uint8 )
            image = cv2.imdecode( npArray, cv2.CV_LOAD_IMAGE_COLOR )
                
        except ( TypeError, cv2.error ):
            pass    # Ignore exceptions which arise from invalid data
        
        return image
    
    #-----------------------------------------------------------------------------------------------
    def _process_raw_motion_vectors_data( self, motion_vectors_data ):
    
        motion_vectors = None
        
        if motion_vectors_data != None and len( motion_vectors_data ) > 0:
            try:
                motion_vectors_shape = np.fromstring( motion_vectors_data[ :4 ], 
                    dtype=[("width","u2"),("height","u2")] )
                
                motion_vectors = np.fromstring( motion_vectors_data[ 4: ], 
                    dtype=[("x","i1"),("y","i1"),("sad","u2")] )
                motion_vectors.shape = ( motion_vectors_shape[ "height" ], 
                    motion_vectors_shape[ "width" ] )
                    
                #print motion_vectors.shape
            except TypeError:
                pass    # Ignore exceptions which arise from invalid data
                        
        return motion_vectors
            
    
