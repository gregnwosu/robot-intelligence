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

PIN_FUNC_INACTIVE = "inactive"
PIN_FUNC_DIGITAL_READ = "digital"
PIN_FUNC_ANALOG_READ = "analog"
PIN_FUNC_ULTRASONIC_READ = "ultrasonic"
ENCODER_TYPE_SINGLE_OUTPUT = "single_output"
ENCODER_TYPE_QUADRATURE = "quadrature"

#---------------------------------------------------------------------------------------------------
class SensorConfiguration:
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, configD12=PIN_FUNC_ULTRASONIC_READ, 
        configD13=PIN_FUNC_INACTIVE, 
        configA0=PIN_FUNC_ANALOG_READ, configA1=PIN_FUNC_ANALOG_READ,
        configA2=PIN_FUNC_ANALOG_READ, configA3=PIN_FUNC_ANALOG_READ,
        configA4=PIN_FUNC_ANALOG_READ, configA5=PIN_FUNC_ANALOG_READ,
        leftEncoderType=ENCODER_TYPE_QUADRATURE, rightEncoderType=ENCODER_TYPE_QUADRATURE ):
            
        self.configD12 = configD12
        self.configD13 = configD13
        self.configA0 = configA0
        self.configA1 = configA1
        self.configA2 = configA2
        self.configA3 = configA3
        self.configA4 = configA4
        self.configA5 = configA5
        self.leftEncoderType = leftEncoderType
        self.rightEncoderType = rightEncoderType

    #-----------------------------------------------------------------------------------------------
    @classmethod
    def createFromDictionary( self, dictionary ):
        
        sensorConfiguration = SensorConfiguration()
        
        try:
            if type( dictionary ) == dict:
                if "configD12" in dictionary:
                    value = str( dictionary[ "configD12" ] ).lower()
                    
                    if value == PIN_FUNC_DIGITAL_READ:
                        sensorConfiguration.configD12 = PIN_FUNC_DIGITAL_READ
                        
                if "configD13" in dictionary:
                    value = str( dictionary[ "configD13" ] ).lower()
                    
                    if value == PIN_FUNC_DIGITAL_READ:
                        sensorConfiguration.configD13 = PIN_FUNC_DIGITAL_READ
                
                for i in range( 6 ):
                    
                    varName = "configA{0}".format( i )
                    
                    if varName in dictionary:
                        value = str( dictionary[ varName ] ).lower()
                        
                        if value == PIN_FUNC_DIGITAL_READ:
                            setattr( sensorConfiguration, varName, PIN_FUNC_DIGITAL_READ )
                            
                if "leftEncoderType" in dictionary:
                    value = str( dictionary[ "leftEncoderType" ] ).lower()
                    
                    if value == ENCODER_TYPE_SINGLE_OUTPUT:
                        sensorConfiguration.leftEncoderType = ENCODER_TYPE_SINGLE_OUTPUT
                        
                if "rightEncoderType" in dictionary:
                    value = str( dictionary[ "rightEncoderType" ] ).lower()
                    
                    if value == ENCODER_TYPE_SINGLE_OUTPUT:
                        sensorConfiguration.rightEncoderType = ENCODER_TYPE_SINGLE_OUTPUT
                        
        except Exception as e:
            logging.error( "Caught exception when parsing Mini Driver SensorConfiguration dictionary" ) 
            logging.error( str( e ) )
            
        return sensorConfiguration
        
    #-----------------------------------------------------------------------------------------------
    def __str__( self ):
        
        return "configD12: {0}\n".format( self.configD12 ) \
            + "configD13: {0}\n".format( self.configD13 ) \
            + "configA0: {0}\n".format( self.configA0 ) \
            + "configA1: {0}\n".format( self.configA1 ) \
            + "configA2: {0}\n".format( self.configA2 ) \
            + "configA3: {0}\n".format( self.configA3 ) \
            + "configA4: {0}\n".format( self.configA4 ) \
            + "configA5: {0}\n".format( self.configA5 ) \
            + "leftEncoderType: {0}\n".format( self.leftEncoderType ) \
            + "rightEncoderType: {0}".format( self.rightEncoderType )
        
