#!/usr/bin/env python

import sys
import time
import serial

platfromControlP15VOn = False
bodyControlP1Identified = False
bodyControlP1IdentifiedPort = None

def bodyControlP1Init(ser):
    ser.write('STOPSHIFTREGRESET')
    ser.write('SHIFTREGENABLE')
    ser.write('HEADLEDON')
    time.sleep(1.0)
    ser.write('HEADLEDOFF')
    ser.write('WIFIPOWER12VON')

for i in xrange(0, 6):
    ttyACMDeviceName = '/dev/ttyACM{0}'.format(i)
    print ('Inspecting ' + ttyACMDeviceName)
    try:
        ser = serial.Serial(
                port     = ttyACMDeviceName,
                baudrate = 115200,
                parity   = serial.PARITY_NONE,
                stopbits = serial.STOPBITS_ONE,
                bytesize = serial.EIGHTBITS,
                timeout  = 1)
        ser.write('GETID')
        x = ser.readline()
        x = x.rstrip('\r\n')
        print x
        if x == 'BODY-CONTROL-P1':
            print 'BODY-CONTROL-P1 found'
            bodyControlP1Identified = True
            bodyControlP1IdentifiedPort = ser
            if platfromControlP15VOn == True:
                bodyControlP1Init(ser)

        if x == 'PLATFORM-CONTROL-P1':
            print 'PLATFORM-CONTROL-P1 found'
            ser.write('DCDC5VENABLEON')
            time.sleep(1.0)
            platfromControlP15VOn = True
            if bodyControlP1Identified == True: 
                bodyControlP1Init(bodyControlP1IdentifiedPort)

        time.sleep(0.5)
    except:
        print("Unexpected error:", sys.exc_info()[0])
