#Importing packages
import cv2
import RPi.GPIO as gpio
import time
import numpy as np
from Utils.motion import *
from Utils.sonar import *
from Utils.imu import *
import io
import socket
import struct
import time
import picamera
import sys

if __name__ == "__main__":
    t1 = time.time()
    startNode = [0,0,0]
    #Initialize Serial Port and PWM Pins
    print('\n-------->Program Start<--------')
    print('\n-------->Initializing Serial and GPIO pins<--------')
    init()
    # print('\n-------->Current Parameters<--------')
    # startNode[2] = getYaw()
    # print('\nCurrent Position: ',startNode)
    # newPos = startNode
    print('\n-------->Script Start<--------')
    txtfilename = 'positiondata_'+str(time.time())+'.txt'
    #f = open(txtfilename,'a')
    
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # client_socket.connect(('192.168.137.80', 8000)) #LAPTOP IP
    client_socket.connect(('10.104.44.73', 8000)) #LAPTOP IP
    #client_socket.connect((sys.argv[1], int(sys.argv[2])))
    print('\n-------->Connected to the server<--------')
    connection = client_socket.makefile('wb')
    try: 
        with picamera.PiCamera() as camera:
            camera.resolution = (640,480) #(1080, 810)
            camera.vflip=True
            camera.hflip=True
            print("starting Camera...........")
            time.sleep(2)
            stream = io.BytesIO()        
            for foo in camera.capture_continuous(stream, 'jpeg'):
                connection.write(struct.pack('<L', stream.tell()))
                connection.flush()
                stream.seek(0)
                connection.write(stream.read())
                stream.seek(0)
                stream.truncate()

                response = client_socket.recv(1024).decode()
                print('Received Response:', response)
                dist = avg_dist()
                print('\nDistance:',dist)
                if int(response) == 1:
                    print('Object Found')
                    pivotleft(40)
                    forward(25)
                    pivotright(40)
                else:
                    if dist<35:
                        pivotleft(90)
                    forward(10)
            
    except KeyboardInterrupt:
        print('\n-------->Interrupt detected<--------')
    finally:
        t2 = time.time()
        print('\nRuntime: ',t2-t1,' seconds')
        print('\n-------->Cleaning up pins<--------')
        gameover()
        gpio.cleanup()
        cv2.destroyAllWindows()
        #f.close()
        print('\n-------->Cleaning up successful<--------')
        connection.close()
        client_socket.close()
        print('\n-------->Program End<--------')
    


