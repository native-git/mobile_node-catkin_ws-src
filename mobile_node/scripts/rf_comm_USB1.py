import serial
import rospy


ser = serial.Serial()
ser.port = '/dev/ttyUSB1'
ser.baudrate = 38400
ser.open()

response = ser.readline()

print response

ser.close()