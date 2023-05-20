import serial
import time
#Identify serial connection
ser = serial.Serial('/dev/ttyUSB0',9600)
def getYaw():
    #ser = serial.Serial('/dev/ttyUSB0',9600)
    count = 0
    while True:
        if(ser.in_waiting > 0):
            count += 1
            line = ser.readline()
            # Avoid first n-lines of serial info
            if count > 10:
                # Strip serial stream of characters
                line = line.rstrip().lstrip()
                line = str(line)
                line = line.strip("'")
                line = line.strip("b'")
                line = float(line)
                #print(count, line,"\n")
                return line
            