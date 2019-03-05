import serial			#used for data transmission/receiving
import time			#used for timing frquency
from pymavlink import mavutil	#mavlink command library

port = "/dev/ttyACMO" #set to port teensey is connected to
rate = 9600

##Setup##

#open up serial port
ser = serial.Serial(port, rate)
ser.flushInput()

#get timing (5 seconds) for enviormental pull
t_enviro_pull = time.time() + 5

#define mavlink parameters



##End Setup##

#loop until terminated by Pi
while True:    
	
	if t_enviro_pull == time.time():
		#ask teensey for enviromental data
		ser.Write(1)
		
		#store enviromental data
		temp = ser.readline()
		altitude = ser.readline()
		pressure = ser.readline()
		
		#debug
		print(temp)
		print(altitude)
		print(pressure)
		
		#reset "timer"
		t_enviro_pull = time.time() + 5
	
	#read in distance data
	distance_front = ser.readline()
	distance_top = ser.readline()
	
	#send data to pixhawk via mavlink
