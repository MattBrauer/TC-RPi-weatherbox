# Classes and functions for interfacing RPi via RF24 to an Arduino "weatherbox" station.

from __future__ import print_function
from RF24 import *
from struct import *

import time
import operator
import syslog

import weewx.drivers
import weewx.wxformulas
import weeutil.weeutil


DRIVER_NAME = 'weatherbox'
DRIVER_VERSION = "0.1"


packetTypes = ["PING","HEAD","WIND","GUST","WIND_2","GUST_10","HUMID","TEMPF","RAININ","RAIN_60","RN_DAY","PRESS","BATT_LVL","LIGHT_LVL"]
packetLabels = ['packetNum','packetType','frame','fValue','iValue'] 
loopLabels = {'WIND':{'iValue':"windDir", 'fValue':"windSpeed"}, # mph instantaneous wind speed (mph); 0-360 instantaneous wind direction
              'GUST':{'iValue':"windGustDir", 'fValue':"windGust"},# mph current wind gust using software specific time period; 0-360 using software specific time period
              'WIND_2':{'iValue':"winddir_avg2m", 'fValue':"windspdmph_avg2m"}, # mph 2 minute average wind speed mph; 0-360 2 minute average wind direction
              'GUST_10':{'iValue':"windgustdir_10m", 'fValue':"windgustmph_10m"}, # mph past 10 minutes wind gust mph; 0-360 past 10 minutes wind gust direction
              'HUMID':{'iValue':"NONE", 'fValue':"humidity"}, # humidity, %
              'TEMPF':{'iValue':"NONE", 'fValue':"tempf"}, # temperature, F
              'RAININ':{'iValue':"rain", 'fValue':"NONE"}, # rain bucket tips accumulated in the past 60 min
              'RAIN_60':{'iValue':"rain60", 'fValue':"NONE"}, # rain bucket tips accumulated in the past 60 min
              'RN_DAY':{'iValue':"dailyrain", 'fValue':"NONE"}, # rain bucket tips so far today (in local time)
              'PRESS':{'iValue':"NONE", 'fValue':"pressure"}, 
              'BATT_LVL':{'iValue':"NONE", 'fValue':"batt_lvl"}, # analog value from 0 to 1023
              'LIGHT_LVL':{'iValue':"NONE", 'fValue':"light_lvl"}} # analog value from 0 to 1023
# need to calculate barometer, altimeter, dewptf

def loader(config_dict, engine):
    return weatherbox(**config_dict[DRIVER_NAME])    

def confeditor_loader():
    return weatherboxConfEditor()

class weatherbox(weewx.drivers.AbstractDevice):
    """Driver for the weatherbox station."""
    
    def __init__(self, **stn_dict) :
        """Initialize an object of type weatherbox.
        
        NAMED ARGUMENTS:
        
        model: Which station model is this?
        [Optional. Default is 'weatherbox']

        cepin: The pin attached to Chip Enable on the RF module
        [Optional. Default is RPI_V2_GPIO_P1_15]

        cspin: The pin attached to Chip Select on the RF module
        [Optional. Default is RPI_V2_GPIO_P1_24]

        spispeed: For RPi, the SPI speed in MHZ
        [Optional. Default is BCM2835_SPI_SPEED_8MHZ]

        stale_wind: Max time wind speed can be used to calculate wind chill
        before being declared unusable. [Optional. Default is 30 seconds]
        
        timeout: How long to wait, in seconds, before giving up on a response from the
        RF24 radio. [Optional. Default is 15 seconds]
        
        wait_before_retry: How long to wait before retrying. [Optional.
        Default is 5 seconds]

        max_tries: How many times to try before giving up. [Optional.
        Default is 3]
        """
        
        self.model             = stn_dict.get('model', 'weatherbox')
        self.record_generation = stn_dict.get('record_generation', 'software')
        self.stale_wind        = float(stn_dict.get('stale_wind', 30.0))
        self.timeout           = float(stn_dict.get('timeout', 15.0))
        self.wait_before_retry = float(stn_dict.get('wait_before_retry', 5.0))
        self.max_tries         = int(stn_dict.get('max_tries', 3))
        self.cepin             = int(stn_dict.get('cepin', RPI_V2_GPIO_P1_15))
        self.cspin             = int(stn_dict.get('cspin', RPI_V2_GPIO_P1_24))
        self.spispeed          = int(stn_dict.get('spispeed', BCM2835_SPI_SPEED_8MHZ))
        self.last_totalRain    = None
        self.radio             = RF24(self.cepin, self.cspin, self.spispeed)

        self.pipes = [0x65646f4e31, 0x65646f4e32]
        self.millis = lambda: int(round(time.time() * 1000))

        self.lastTime = 0
        self.currentTime = 0
        self.totalRain = 0

        self.loopPacket = {}

        self.timelog = {}
        self.packetlog = {}

        self.radio.begin()
        self.radio.enableAckPayload()
        self.radio.enableDynamicPayloads()
        #self.radio.printDetails()

        self.radio.openWritingPipe(self.pipes[1])
        self.radio.openReadingPipe(1,self.pipes[0])
        self.radio.startListening()

    def genPackets(self):
        started_waiting_at = self.millis()
        timeout = False
        while (not self.radio.available()) and (not timeout):
            if (self.millis() - started_waiting_at) > 1000:
                timeout = True

        if not timeout:
            len = self.radio.getDynamicPayloadSize()        
            receive_payload = self.radio.read(len)          
            results = unpack('=BBLfl', receive_payload)
            packet = dict(zip(packetLabels, results))  
            ackMessage = chr(results[0])               
            self.radio.writeAckPayload(1, bytes(ackMessage))
            yield packet

    def genLoopPackets(self):

        for packet in self.genPackets():
            duplicate = True

            if self.currentTime > self.lastTime:
                if len(self.loopPacket) > 2:
                    yield self._fixLoopPacket(self.loopPacket)
                self.loopPacket = {}
                self.packetlog = {}
                self.loopPacket.update({'dateTime':self.currentTime})
                self.loopPacket.update({'usUnits':weewx.US})
                self.lastTime = self.currentTime

            if not self.packetlog.get(packet['packetType'], 0):
                self.packetlog[packet['packetType']] = 1;
                duplicate = False

            if packet['packetType'] > 0:
                if packet['packetType'] == 1:
                    self.timelog[packet['frame']] = self.timelog.get(packet['frame'], time.time())
                else:
                    self.loopPacket.update(self._packetDecode(packet))
                packetTime = self.timelog.get(packet['frame'], time.time())
                self.currentTime = packetTime

    def _packetDecode(self, packet):
        packets = {}
        if loopLabels[packetTypes[packet['packetType']]]['iValue'] != "NONE":
            packetLabel = loopLabels[packetTypes[packet['packetType']]]['iValue']
            packets.update({packetLabel:packet['iValue']})
        if loopLabels[packetTypes[packet['packetType']]]['fValue'] != "NONE":
            packetLabel = loopLabels[packetTypes[packet['packetType']]]['fValue']
            packets.update({packetLabel:packet['fValue']})
        return(packets)

    def _fixLoopPacket(self, loopPacket):
        newLoopPacket = loopPacket
        newLoopPacket.update({'rainRate':newLoopPacket.pop("rain", 0) * 0.011})
        if newLoopPacket.get('rainRate') is not None:
            self.totalRain += loopPacket.get('rain', 0) * 0.011
        newLoopPacket['totalRain'] = self.totalRain
        newLoopPacket.update({'hourRain':newLoopPacket.pop("rain60", 0) * 0.011})
        newLoopPacket.update({'rain24':newLoopPacket.pop("dailyrain", 0) * 0.011})
        newLoopPacket.update({'outTemp':newLoopPacket.pop("tempf", 0)})

        return(newLoopPacket)

    @property
    def hardware_name(self):
        return self.model

class weatherboxConfEditor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return """
[weatherbox]
    # This section is for the Arduino weatherbox

    # The station model, e.g., weatherbox
    model = weatherbox

    # RF24 configuration
    cepin = RPI_V2_GPIO_P1_15
    cspin = RPI_V2_GPIO_P1_24
    spispeed = BCM2835_SPI_SPEED_8MHZ

    # How long a wind record can be used to calculate wind chill (in seconds)
    stale_wind = 30

    # The driver to use:
    driver = weewx.drivers.weatherbox
"""
