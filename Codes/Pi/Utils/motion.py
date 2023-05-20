#Importing packages
import cv2
import RPi.GPIO as gpio
import time
import numpy as np
import serial

IN1 = 31
IN2 = 33
IN3 = 35
IN4 = 37

pwm = 0
pwm1 = 0
pwm2 = 0
pwm3 = 0
pwm4 = 0
ser = serial.Serial('/dev/ttyUSB0',9600)

def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(IN1, gpio.OUT)
    gpio.setup(IN2, gpio.OUT)
    gpio.setup(IN3, gpio.OUT)
    gpio.setup(IN4, gpio.OUT)
    gpio.setup(36,gpio.OUT)
    #Left
    gpio.setup(7,gpio.IN, pull_up_down = gpio.PUD_UP)
    #Right
    gpio.setup(12,gpio.IN, pull_up_down = gpio.PUD_UP)
    global pwm,pwm1,pwm2,pwm3,pwm4
    pwm = gpio.PWM(36,50)
    pwm1 = gpio.PWM(IN1,50);pwm2 = gpio.PWM(IN2,50);pwm3 = gpio.PWM(IN3,50);pwm4 = gpio.PWM(IN4,50)
    pwm.start(3.5)
    
def gameover():
    pwm1.stop();pwm2.stop();pwm3.stop();pwm4.stop()

def pivotright(angle):
    print('Right: ',angle)
    speed= 70 #50
    count = 0
    while True:
        if(ser.in_waiting > 0):
            count += 1
            line = ser.readline()
            if count > 2:
                line = line.rstrip().lstrip();line = str(line)
                line = line.strip("'");line = line.strip("b'")
                yaw = float(line)
                if count == 3:
                    curr_yaw = yaw
                    pwm1.start(speed);pwm2.start(0);pwm3.start(speed);pwm4.start(0)
                    time.sleep(0.1)
                else:
                    if (180 - abs(abs(yaw - curr_yaw)-180)) > (0.9*angle):
                        gameover()
                        #print('State: ',curr_yaw,'Current Yaw: ',yaw,'Required Angle: ',angle)
                        break

def pivotleft(angle):
    print('Left: ',angle)
    speed = 70 #50
    count = 0
    while True:
        if(ser.in_waiting > 0):
            count += 1
            line = ser.readline()
            if count > 2:
                line = line.rstrip().lstrip();line = str(line)
                line = line.strip("'");line = line.strip("b'")
                yaw = float(line)
                if count == 3:
                    curr_yaw = yaw
                    pwm1.start(0);pwm2.start(speed);pwm3.start(0);pwm4.start(speed)
                    time.sleep(0.1)
                else:
                    if (180 - abs(abs(curr_yaw - yaw)-180)) > (0.95*angle):
                        gameover()
                        #print('State: ',curr_yaw,'Current Yaw: ',yaw,'Required Angle: ',angle)
                        break

def gripper_close():
    pwm.ChangeDutyCycle(3.5)
    print('Gripper Close')
    time.sleep(0.1)

def gripper_open():
    pwm.ChangeDutyCycle(7)
    print('Gripper Open')
    time.sleep(0.1)
    
def key_input(event,dist):
    key_press = event
    dist = float(dist)    
    if key_press == 'w':
        print('Forward')
        forward(dist)      
    elif key_press == 's':
        print('Reverse')
        reverse(dist)        
    elif key_press == 'a':
        print('Left')
        pivotleft(dist)   
    elif key_press== 'd':
        print('Right')
        pivotright(dist)
    elif key_press == 'q':
        gripper_close()
    elif key_press == 'r':
        gripper_open()
    else:
        print('Invalid Input')

def forward(distance):
    print('Forward: ',distance)
    counterBR = np.uint64(0);buttonBR = int(0)
    counterFL = np.uint64(0);buttonFL = int(0)
    ticks = (0.98*distance) #gives encoder ticks for x cms
    val_l=40+10
    val_r=34+10
    pwm1.start(val_l);pwm2.start(0);pwm3.start(0);pwm4.start(val_r)
    time.sleep(0.1)
    lastTime = time.time()
    while True:
        if int(gpio.input(12)) != int(buttonBR):
            buttonBR = int(gpio.input(12))
            counterBR += 1
        if int(gpio.input(7)) != int(buttonFL):
            buttonFL = int(gpio.input(7))
            counterFL += 1
        timeInterval = time.time() - lastTime
        if (timeInterval>0.3):
            count_diff = counterFL - counterBR
            if count_diff>0:
                val_r = max(min(90,1.03*val_r),0)
                pwm4.ChangeDutyCycle(val_r)
                #print('\nNew Speeds: ',val_l,val_r)
            elif count_diff<0:
                val_l = max(min(80,1.03*val_l),0)
                pwm1.ChangeDutyCycle(val_l)
                #print('\nNew Speeds: ',val_l,val_r)
            #print('Left: ',counterFL,'Right: ',counterBR)
            lastTime = time.time()
        if (counterBR+counterFL)/2 == 0.9*ticks:
            val_l = max(min(90,0.7*val_l),40)
            val_r = max(min(80,0.7*val_r),36)
            pwm1.ChangeDutyCycle(val_l)
            pwm4.ChangeDutyCycle(val_r)
            #print('\nNew Speeds: ',val_l,val_r)
        if counterBR >= ticks and counterFL >= ticks:
            gameover()
            break

def reverse(distance):
    print('Reverse: ',distance)
    counterBR = np.uint64(0);buttonBR = int(0)
    counterFL = np.uint64(0);buttonFL = int(0)
    ticks = (0.98*distance) #gives encoder ticks for x cms
    val_l=40+20
    val_r=34+20
    pwm1.start(0);pwm2.start(val_l);pwm3.start(val_r);pwm4.start(0)
    time.sleep(0.1)
    lastTime = time.time()
    while True:
        if int(gpio.input(12)) != int(buttonBR):
            buttonBR = int(gpio.input(12))
            counterBR += 1
        if int(gpio.input(7)) != int(buttonFL):
            buttonFL = int(gpio.input(7))
            counterFL += 1
        timeInterval = time.time() - lastTime
        if (timeInterval>0.3):
            count_diff = counterFL - counterBR
            if count_diff>0:
                val_r = max(min(90,1.03*val_r),0)
                pwm3.ChangeDutyCycle(val_r)
                #print('\nNew Speeds: ',val_l,val_r)
            elif count_diff<0:
                val_l = max(min(80,1.03*val_l),0)
                pwm2.ChangeDutyCycle(val_l)
                #print('\nNew Speeds: ',val_l,val_r)
            #print('Left: ',counterFL,'Right: ',counterBR)
            lastTime = time.time()
        if (counterBR+counterFL)/2 == 0.9*ticks:
            val_l = max(min(90,0.7*val_l),40)
            val_r = max(min(80,0.7*val_r),40)
            pwm2.ChangeDutyCycle(val_l)
            pwm3.ChangeDutyCycle(val_r)
            #print('\nNew Speeds: ',val_l,val_r)
        if counterBR >= ticks and counterFL >= ticks:
            gameover()
            break
