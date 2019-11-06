import settings
import serial
ser = serial.Serial(settings.SERIAL_PORT, 9600)
port = settings.SERIAL_PORT

print("connected to: " + ser.portstr)
count=1

while True:
    for line in ser.read():
        print(chr(line),end='')

ser.close()