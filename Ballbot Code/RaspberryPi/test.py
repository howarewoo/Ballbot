import serial

ser = serial.Serial('/dev/ttyACM0',38400)
s = [0]
while True:
	read_serial=ser.readline()
	s[0] = str(int (ser.readline(),16))
	print s[0]
	print read_serial
