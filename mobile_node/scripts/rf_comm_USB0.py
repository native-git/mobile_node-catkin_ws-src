#!/usr/bin/env python

import serial

ser = serial.Serial()
ser.port = '/dev/ttyUSB0'
ser.baudrate = 38400
ser.open()
print "hello"
ser.write(b'hello')
print "Message Sent"

msg = "HELLO!@@$$$^%&\r\n"

ser.write(msg.encode())

ser.close()