#! /usr/bin/env python

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

import logging
import os.path
import json
import mini_driver

# TODO: This file shares a /lot/ of code with robot_config.py in raspberry_pi_camera_bot
# it might be nice to find a way to share this at some point

#--------------------------------------------------------------------------------------------------- 
class RobotConfig:

    MIN_PULSE_WIDTH = 400
    MAX_PULSE_WIDTH = 2600

    #-----------------------------------------------------------------------------------------------
    def __init__( self, configFilename=None ):
        
        self.scriptPath = os.path.dirname( __file__ )
        
        self.panPulseWidthMin = 700
        self.panPulseWidthMax = 2100
        self.tiltPulseWidthMin = 550
        self.tiltPulseWidthMax = 2400
        self.usePresetMotorSpeeds = True
        self.customMaxAbsMotorSpeed = 50.0
        self.customMaxAbsTurnSpeed = 30.0
        self.leftMotorScale = 1.0
        self.miniDriverSensorConfiguration = mini_driver.SensorConfiguration()
        self.piSensorModuleName = "sensors.default_sensor_reader"
        
        if configFilename != None:
            self.tryToLoadConfigFile( configFilename )
        
    #-----------------------------------------------------------------------------------------------
    def tryToLoadConfigFile( self, configFilename ):
        
        absConfigFilename = os.path.abspath( configFilename )
        
        if os.path.exists( absConfigFilename ):
            try:
                with open( absConfigFilename ) as configFile:
                    
                    configDict = json.load( configFile )
                    self.readDataFromConfigDict( configDict )
                    
            except Exception as e:
                logging.warning( "Unable to load config file. Exception was " + str( e ) )
    
    #-----------------------------------------------------------------------------------------------
    def readDataFromConfigDict( self, configDict ):
        
        if "panPulseWidthMin" in configDict:
            self.panPulseWidthMin = self.parsePulseWidth( 
                configDict[ "panPulseWidthMin" ], self.panPulseWidthMin )
        
        if "panPulseWidthMax" in configDict:
            self.panPulseWidthMax = self.parsePulseWidth( 
                configDict[ "panPulseWidthMax" ], self.panPulseWidthMax )
        
        if "tiltPulseWidthMin" in configDict:
            self.tiltPulseWidthMin = self.parsePulseWidth( 
                configDict[ "tiltPulseWidthMin" ], self.tiltPulseWidthMin )
        
        if "tiltPulseWidthMax" in configDict:
            self.tiltPulseWidthMax = self.parsePulseWidth( 
                configDict[ "tiltPulseWidthMax" ], self.tiltPulseWidthMax )
                
        if "usePresetMotorSpeeds" in configDict:
            data = configDict[ "usePresetMotorSpeeds" ]
            self.usePresetMotorSpeeds = (str( data ).lower() == "true")
        
        if "customMaxAbsMotorSpeed" in configDict:
            self.customMaxAbsMotorSpeed = self.parseDutyCycle( 
                configDict[ "customMaxAbsMotorSpeed" ] )
                
        if "customMaxAbsTurnSpeed" in configDict:
            self.customMaxAbsTurnSpeed = self.parseDutyCycle( 
                configDict[ "customMaxAbsTurnSpeed" ] )
            
        if "leftMotorScale" in configDict:
            self.leftMotorScale = self.parseDutyCycle( 
                configDict[ "leftMotorScale" ] )
                
        if "miniDriverSensorConfiguration" in configDict:
            self.miniDriverSensorConfiguration = mini_driver.SensorConfiguration.createFromDictionary( 
                configDict[ "miniDriverSensorConfiguration" ] )
                
        if "piSensorModuleName" in configDict:
            self.piSensorModuleName = str( configDict[ "piSensorModuleName" ] )
    
    #-----------------------------------------------------------------------------------------------
    def parsePulseWidth( self, inputData, defaultValue=MIN_PULSE_WIDTH ):
        
        result = defaultValue
        
        try:
            result = int( inputData )
        except Exception:
            pass
        
        return max( self.MIN_PULSE_WIDTH, min( result, self.MAX_PULSE_WIDTH ) )
    
    #-----------------------------------------------------------------------------------------------
    def parseDutyCycle( self, inputData ):
        
        result = 0.0
        
        try:
            result = float( inputData )
        except Exception:
            pass
        
        return max( 0.0, min( result, 100.0 ) )
    
    #-----------------------------------------------------------------------------------------------
    def writeConfigFile( self, configFilename ):
        
        configDict = self.getConfigDict()
        
        absConfigFilename = os.path.abspath( configFilename )
        
        if not os.path.exists( absConfigDir ):
            os.makedirs( absConfigDir )
            
        with open( absConfigFilename, "w" ) as configFile:
            
            json.dump( configDict, configFile, indent=4, default=lambda o: o.__dict__ )
        
    #-----------------------------------------------------------------------------------------------
    def getConfigDict( self ):    
    
        configDict = {
            "panPulseWidthMin" : self.panPulseWidthMin,
            "panPulseWidthMax" : self.panPulseWidthMax,
            "tiltPulseWidthMin" : self.tiltPulseWidthMin,
            "tiltPulseWidthMax" : self.tiltPulseWidthMax,
            "usePresetMotorSpeeds" : self.usePresetMotorSpeeds,
            "customMaxAbsMotorSpeed" : self.customMaxAbsMotorSpeed,
            "customMaxAbsTurnSpeed" : self.customMaxAbsTurnSpeed,
            "leftMotorScale" : self.leftMotorScale,
            "miniDriverSensorConfiguration" : self.miniDriverSensorConfiguration,
            "piSensorModuleName" : self.piSensorModuleName,
        }
        
        return configDict
        
    #-----------------------------------------------------------------------------------------------
    def __str__( self ):
        
        return "panPulseWidthMin: {0}\n".format( self.panPulseWidthMin ) \
            + "panPulseWidthMax: {0}\n".format( self.panPulseWidthMax ) \
            + "tiltPulseWidthMin: {0}\n".format( self.tiltPulseWidthMin ) \
            + "tiltPulseWidthMax: {0}\n".format( self.tiltPulseWidthMax ) \
            + "usePresetMotorSpeeds: {0}\n".format( self.usePresetMotorSpeeds ) \
            + "customMaxAbsMotorSpeed: {0}\n".format( self.customMaxAbsMotorSpeed ) \
            + "customMaxAbsTurnSpeed: {0}\n".format( self.customMaxAbsTurnSpeed ) \
            + "leftMotorScale: {0}\n".format( self.leftMotorScale ) \
            + "miniDriverSensorConfiguration: {0}\n".format( self.miniDriverSensorConfiguration ) \
            + "piSensorModuleName: {0}".format( self.piSensorModuleName )
