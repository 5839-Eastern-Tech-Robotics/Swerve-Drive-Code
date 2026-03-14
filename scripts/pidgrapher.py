import serial

ser = serial.Serial('/dev/ttyACM1')
while True:
    print(ser.readline().decode())
