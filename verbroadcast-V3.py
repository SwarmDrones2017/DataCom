#!/usr/bin/env python

from __future__ import division
import signal
import socket
import time
import string
import sys
import getopt
import math
import threading
from array import *
import smbus
import select
import os
import struct
import logging
import RPi.GPIO as GPIO
import subprocess
from datetime import datetime
import shutil
import ctypes
from ctypes.util import find_library
import random
import nmap

############################################################################################
#
#  Adafruit i2c interface enhanced with performance / error handling enhancements
#
############################################################################################

class I2C:

        def __init__(self, address, bus=smbus.SMBus(1)):
                self.address = address
                self.bus = bus
                self.misses = 0

        def reverseByteOrder(self, data):
                "Reverses the byte order of an int (16-bit) or long (32-bit) value"
                # Courtesy Vishal Sapre
                dstr = hex(data)[2:].replace('L','')
                byteCount = len(dstr[::2])
                val = 0
                for i, n in enumerate(range(byteCount)):
                        d = data & 0xFF
                        val |= (d << (8 * (byteCount - i - 1)))
                        data >>= 8
                return val

        def write8(self, reg, value):
                "Writes an 8-bit value to the specified register/address"
                while True:
                        try:
                                self.bus.write_byte_data(self.address, reg, value)
                                break
                        except IOError, err:
                                self.misses += 1
                                time.sleep(0.0001)

        def writeList(self, reg, list):
                "Writes an array of bytes using I2C format"
                while True:
                        try:
                                self.bus.write_i2c_block_data(self.address, reg, list)
                                break
                        except IOError, err:
                                self.misses += 1
                                time.sleep(0.0001)

        def readU8(self, reg):
                "Read an unsigned byte from the I2C device"
                while True:
                        try:
                                result = self.bus.read_byte_data(self.address, reg)
                                return result
                        except IOError, err:
                                self.misses += 1
                                time.sleep(0.0001)

        def readS8(self, reg):
                "Reads a signed byte from the I2C device"
                while True:
                        try:
                                result = self.bus.read_byte_data(self.address, reg)
                                if (result > 127):
                                        return result - 256
                                else:
                                        return result
                        except IOError, err:
                                self.misses += 1
                                time.sleep(0.0001)

        def readU16(self, reg):
                "Reads an unsigned 16-bit value from the I2C device"
                while True:
                        try:
                                hibyte = self.bus.read_byte_data(self.address, reg)
                                result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg+1)
                                return result
                        except IOError, err:
                                self.misses += 1
                                time.sleep(0.0001)

        def readS16(self, reg):
                "Reads a signed 16-bit value from the I2C device"
                while True:
                        try:
                                hibyte = self.bus.read_byte_data(self.address, reg)
                                if (hibyte > 127):
                                        hibyte -= 256
                                result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg+1)
                                if result == 0x7FFF or result == 0x8000:
                                        time.sleep(0.0005)
                                else:
                                        return result
                        except IOError, err:
                                self.misses += 1
                                time.sleep(0.0001)
                                
        def readList(self, reg, length):
                "Reads a a byte array value from the I2C device"
                while True:
                        try:
                                result = self.bus.read_i2c_block_data(self.address, reg, length)
                                return result
                        except IOError, err:
                                self.misses += 1
                                time.sleep(0.0001)

        def getMisses(self):
                return self.misses

############################################################################################
#
#  Ultrasonic range finder
#
############################################################################################

class SRF02:
        i2c = None

        #Reisters/etc
        __SRF02_RA_CONFIG = 0x00
        __SRF02_RA_RNG_HI = 0x02
        __SRF02_RA_RNG_LO = 0x03
        __SRF02_RA_AUTO_HI = 0x04
        __SRF02_RA_AUTO_LO = 0x05

        def __init__(self, address):
                self.i2c = I2C(address)


        def pingProximity(self):
                #---------------------------------------------------------------------------
                # Set up range units as centimeters and ping
                #---------------------------------------------------------------------------
                self.i2c.write8(self.__SRF02_RA_CONFIG, 0x51)

        def checkDataReady(self):
                #---------------------------------------------------------------------------
                # Check if data is available
                #---------------------------------------------------------------------------
                rc = self.i2c.read8(self.__SRF02_RA_CONFIG)
                if rc == 0xFF:
                        return True
                else:
                        return False

        def readProximity(self):
                #---------------------------------------------------------------------------
                # Read proximity
                #---------------------------------------------------------------------------
                range = self.i2c.readU16(self.__SRF02_RA_RNG_HI)
                return range
        

#------------------------------------------------------------
# Set up the shutdown handler
#------------------------------------------------------------
def ShutdownHandler(signal, frame):
        global keep_looping
        keep_looping = False


GPIO_BUTTON = 18
GPIO.setmode(GPIO.BOARD)
GPIO.setup(GPIO_BUTTON, GPIO.IN, GPIO.PUD_DOWN)


#------------------------------------------------------------
# Let's go!
#------------------------------------------------------------

#Adress of sensors
nrange = SRF02(0x70)
wrange = SRF02(0x71)
srange = SRF02(0X72)
erange = SRF02(0X73)

#Bool for no send several time 
alreadysendn = False
alreadysendw = False
alreadysends = False
alreadysende = False

#Adress and port using for communication
port = 55555;
adresse = ('',55555)

#Creation of UDP packet
try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(1)
	s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
except socket.error:
        print 'Failed to create socket'
        sys.exit()

signal.signal(signal.SIGINT, ShutdownHandler)

isPresent = False
#Infinite loop
while 1:
        
        keep_looping = True
        reponse = ''
        #Send the message Télephone in broadcast on the network
	try :
                if isPresent == False :
			addr = '255.255.255.255'
                	msg = 'Telephone ?\n'
                        s.sendto(msg, (addr, port))
                
                reponse, adresse = s.recvfrom(1024)
		print reponse
	#If no answer, re-send the message in broadacst on the network
        except socket.timeout :
                print "pas de ack"
		isPresent = False
	                       
        if reponse == 'Oui\n' :
                if isPresent == False :
			#Take adress of answer
                        addr_smartphone = adresse
                        isPresent = True
                        
        if isPresent == True :
                                                
                nrange.pingProximity()
                wrange.pingProximity()
                srange.pingProximity()
                erange.pingProximity()
		
                time.sleep(0.1)
                nproximity = nrange.readProximity()
                wproximity = wrange.readProximity()
                sproximity = srange.readProximity()
                eproximity = erange.readProximity()
                
                c = "Sensor "
		
		#Send distance if less than distance threshold
                if nproximity <= 150:
                                                        
                        c += str(nproximity)+":n,"
                        alreadysendn = False
                else:
                        if(alreadysendn == False):
                                c += "151:n,"
                                alreadysendn = True
                                                 
                if wproximity <= 150:
                        c += str(wproximity)+":w,"
                        alreadysendw = False
                else:
                        if(alreadysendw == False):
                                c += "151:w,"
                                alreadysendw = True
                                                      
                if sproximity <= 150:
                                        
                        c += str(sproximity)+":s,"
                        alreadysends = False
                else:
                        if(alreadysends == False):
                                c += "151:s,"
                                alreadysends = True
                                                 
                if eproximity <= 150:
                                        
                        c += str(eproximity)+":e,"
                        alreadysende = False
                else:
                        if(alreadysende == False):
                                c += "151:n,"
                                alreadysende = True
				
                c += "\n"
                if(c != "Sensor \n"):
			#Send value of sensors
                        s.sendto(c, adresse)
		
		#Display of data
                print "North "+ str(nproximity)+" cm"
                print "West " + str(wproximity) + " cm"
                print "South " + str(sproximity) + " cm"
                print "Est " + str(eproximity) + " cm"
