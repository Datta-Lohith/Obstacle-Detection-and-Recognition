#Importing packages
import cv2
import RPi.GPIO as gpio
import time
import numpy as np
import math
import serial
import cvzone
from ultralytics import YOLO

# Define the parameters
object_height = 6   # Height of the object in cm
sensor_height = 2.76  # Height of the camera sensor in mm
focal_length = 3.04  # Focal length of the camera lens in mm
image_height = 480  # Height of the image captured by the camera in pixels
pixel_size = 0.0014  # Pixel size of the camera in mm
gpio.setwarnings(False)

trig = 16
echo = 18
img_x, img_y = 320,240
startNode = [270,30,0]
goalNode = [50,50]
#Identify serial connection
ser = serial.Serial('/dev/ttyUSB0',9600)

IN1 = 31
IN2 = 33
IN3 = 35
IN4 = 37

pwm = 0
pwm1 = 0
pwm2 = 0
pwm3 = 0
pwm4 = 0

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

def calcNewPos(oldPosition,distance=0,action = 'q'):
    angleDegrees = getYaw()
    angleRadians = math.radians(angleDegrees)
    if action =='w': distance = distance
    elif action =='s': distance = -distance
    else: distance = 0
    #print(distance)
    x = oldPosition[0] + distance * math.sin(angleRadians)
    y = oldPosition[1] + distance * math.cos(angleRadians)
    newPos = (x, y, angleDegrees)
    print('>>>New Position is: ', newPos)
    return newPos

def go2Pos(curPos,goalPos = [100,100,0]):
    print(f'\nGoing to position {goalPos}')
    xDiff = goalPos[0]-curPos[0]
    yDiff = goalPos[1]-curPos[1]
    goalOrient = math.degrees(math.atan2(xDiff, yDiff))
    dist = (xDiff**2 + yDiff**2)**0.5
    rotation = goalOrient - curPos[2] 
    if rotation > 180: rotation -= 360
    elif rotation < -180: rotation += 360
    if rotation>0:
        #key_input('d',abs(rotation))
        pivotright(abs(rotation))
    elif rotation<0:
        #key_input('a',abs(rotation))
        pivotleft(abs(rotation))
    #key_input('w',dist)
    forward(dist)
    #print('Distance, Rotation: ',dist,rotation)
    newPos = calcNewPos(curPos,dist,'w')
    turn2Ang(goalPos[2])
    newPos = calcNewPos(newPos,dist,'d')
    return newPos

def turn2Ang(goalAngle):
    print(f'\nTurning to Angle {goalAngle}')
    rotation = goalAngle - getYaw()
    if rotation > 180:
        rotation -= 360
    elif rotation < -180:
        rotation += 360
    if rotation>0:
        print('Left')
        pivotright(abs(rotation))
    elif rotation<0:
        print('Right')
        pivotleft(abs(rotation))

def reOrient(pos):
    print(f'\nReOrient at DropZone {pos}')
    turn2Ang(270)
    time.sleep(0.5)
    x = avg_dist()
    print('\n X:',x)
    turn2Ang(180)
    time.sleep(0.5)
    y = avg_dist()
    print('\n Y:',y)
    if not isinstance(x,float):x = pos[0]
    if not isinstance(y,float):y = pos[1]
    newPos = (x,y,getYaw())
    print('>>>Re-Oriented Position is: ', newPos)
    return newPos

def go2SearchAngle(i):
    searchAngles = [25,50,75,90,335,0]
    turn2Ang(searchAngles[i])
    i+=1
    return i

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

def forward_nopid(distance):
    counterBR = np.uint64(0)
    buttonBR = int(0)
    counterFL = np.uint64(0)
    buttonFL = int(0)
    ticks = (0.98*distance) #gives encoder ticks for x cms
    val_l=40
    val_r=36
    pwm1.start(val_l);pwm2.start(0);pwm3.start(0);pwm4.start(val_r)
    time.sleep(0.1)
    while True:
        if int(gpio.input(12)) != int(buttonBR):
            buttonBR = int(gpio.input(12))
            counterBR += 1
        if int(gpio.input(7)) != int(buttonFL):
            buttonFL = int(gpio.input(7))
            counterFL += 1
#		if counterBR >= ticks: break
#		if counterFL >= ticks: break
        if counterBR >= ticks and counterFL >= ticks:
            print('Right: ',counterBR,'Left: ',counterFL)
            gameover()
            break

def reverse_nopid(distance):
    counterBR = np.uint64(0)
    buttonBR = int(0)
    counterFL = np.uint64(0)
    buttonFL = int(0)
    ticks = (0.98*distance) #gives encoder ticks for x cms
    val_l=40
    val_r=36
    pwm1.start(0);pwm2.start(val_l);pwm3.start(val_r);pwm4.start(0)
    time.sleep(0.1)
    while True:
        if int(gpio.input(12)) != int(buttonBR):
            buttonBR = int(gpio.input(12))
            counterBR += 1
        if int(gpio.input(7)) != int(buttonFL):
            buttonFL = int(gpio.input(7))
            counterFL += 1
#		if counterBR >= ticks: break
#		if counterFL >= ticks: break
        if counterBR >= ticks and counterFL >= ticks:
            print('\nNew Speeds: ',val_l,val_r)
            print('Right: ',counterBR,'Left: ',counterFL)
            gameover()
            break

def pivotright(angle):
    print('Right: ',angle)
    speed=50
    count = 0
    while True:
        if(ser.in_waiting > 0):
            count += 1
            line = ser.readline()
            if count > 10:
                line = line.rstrip().lstrip();line = str(line)
                line = line.strip("'");line = line.strip("b'")
                yaw = float(line)
                if count == 11:
                    curr_yaw = yaw
                    pwm1.start(speed);pwm2.start(0);pwm3.start(speed);pwm4.start(0)
                    time.sleep(0.1)
                else:
                    #print('State: ',curr_yaw,'Current Yaw: ',yaw,'Required Angle: ',angle)
                    if (abs((180 - abs(abs(yaw - curr_yaw)-180)) > (0.7*angle)))and (abs((180 - abs(abs(yaw - curr_yaw)-180)) < (0.8*angle))):
                        speed = 40
                        pwm1.ChangeDutyCycle(speed)
                        pwm3.ChangeDutyCycle(speed)
                        #print('\nNew Speeds: ',speed)
                    elif (180 - abs(abs(yaw - curr_yaw)-180)) > (0.90*angle):
                        gameover()
                        #print('State: ',curr_yaw,'Current Yaw: ',yaw,'Required Angle: ',angle)
                        break

def pivotleft(angle):
    print('Left: ',angle)
    speed = 50
    count = 0
    while True:
        if(ser.in_waiting > 0):
            count += 1
            line = ser.readline()
            if count > 10:
                line = line.rstrip().lstrip();line = str(line)
                line = line.strip("'");line = line.strip("b'")
                yaw = float(line)
                if count == 11:
                    curr_yaw = yaw
                    pwm1.start(0);pwm2.start(speed);pwm3.start(0);pwm4.start(speed)
                    time.sleep(0.1)
                else:
                    #print('State: ',curr_yaw,'Current Yaw: ',yaw,'Required Angle: ',angle)
                    if (abs((180 - abs(abs(curr_yaw - yaw)-180)) > (0.7*angle))) and (abs((180 - abs(abs(curr_yaw - yaw)-180)) < (0.8*angle))):
                        speed = 40
                        pwm2.ChangeDutyCycle(speed)
                        pwm4.ChangeDutyCycle(speed)
                        #print('\nNew Speeds: ',speed)
                    elif (180 - abs(abs(curr_yaw - yaw)-180)) > (0.90*angle):
                        gameover()
                        #print('State: ',curr_yaw,'Current Yaw: ',yaw,'Required Angle: ',angle)
                        break

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
    else:
        print('Invalid Input')

def forward(distance):
    print('Forward: ',distance)
    counterBR = np.uint64(0);buttonBR = int(0)
    counterFL = np.uint64(0);buttonFL = int(0)
    ticks = (0.98*distance) #gives encoder ticks for x cms
    val_l=40
    val_r=34
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
    val_l=40
    val_r=34
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

def get_frame():
    #cap = cv2.VideoCapture(-1, cv2.CAP_V4L)
    cap = cv2.VideoCapture(0)
    model=YOLO('../yolo-weights/best (5).pt')
    classnames=['ball', 'bottle', 'mug', 'shoes', 'bag']
    while True:
        ret, frame=cap.read()
        results=model(frame,stream=True)
        for r in results:
            boxes=r.boxes
            for box in boxes:
                x1,y1,x2,y2=box.xyxy[0]
                x1,y1,x2,y2=int(x1),int(y1),int(x2),int(y2)
                w=x2-x1
                h=y2-y1
            
                # x,y,w,h= box.xywh[0]
                bbox=int(x1),int(y1),int(w),int(h)
                # print(x1,y1,x2,y2)
                # cv2.rectangle(frame,(x1,y1),(x2,y2),(0,0,255),5)
                # cv2.rectangle(frame,(x1,y1),(x2,y2),(0,0,255),5)
                cvzone.cornerRect(frame,bbox)
                confidence=math.ceil((box.conf[0])*100)/100
                
                cls=int(box.cls[0])
                # if confidence>0.50:
                cvzone.putTextRect(frame,f'{classnames[cls]}' f'{confidence}',(max(0,x1),max(30,y1)))
        
        cv2.imshow("front cam",frame)
        if cv2.waitKey(1)== ord('q'):
            break

    cap.release()

# def analyze_frame(img,color):
#     #img_x,img_y = int(img.shape[1]/2),int(img.shape[0]/2)
#     img = cv2.line(img,(int(img_x-40),int(img_y)),(int(img_x+40),int(img_y)),(0,0,0),1)
#     img = cv2.line(img,(int(img_x),int(img_y-40)),(int(img_x),int(img_y+40)),(0,0,0),1)
#     color_lower = np.array([[150,80,0],[40,40,0],[90,110,0]])
#     color_upper = np.array([[179,255,255],[90,255,255],[120,255,255]])
#     hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV) 
#     mask = cv2.inRange(hsv,color_lower[color],color_upper[color])
#     #masked_img = cv2.bitwise_and(img,img,mask=mask)
#     contours,_=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#     center = 0
#     h = 0
#     if contours:
#         #areas = [cv2.contourArea(c) for c in contours]
#         #max_index = np.argmax(areas)
#         #cnt=contours[max_index]
#         cnt = max(contours, key=cv2.contourArea)
#         area = cv2.contourArea(cnt)
#         if area>300:
#             x, y, w, h = cv2.boundingRect(cnt)
#             center = (int((x+x+w)/2),int((y+y+h)/2))
#             cv2.rectangle(img, (x,y),(x+w,y+h), (0,0,255), 3)
#             cv2.circle(img,center,1,(255,0,0),2)
#             print(center)
#     return img,center,h

def distance():
	gpio.setmode(gpio.BOARD)
	gpio.setup(trig, gpio.OUT)
	gpio.setup(echo, gpio.IN)
	
	gpio.output(trig, True)
	time.sleep(0.00001)
	gpio.output(trig, False)
	
	pulse_start = time.time()
	pulse_end = time.time()

	while gpio.input(echo) ==0:
		pulse_start = time.time()

	while gpio.input(echo) == 1:
		pulse_end = time.time()

	pulse_duration = pulse_end - pulse_start

	distance = pulse_duration*17150
	distance = round(distance, 2)

	#gpio.cleanup()
	return distance

def avg_dist():
	d = 0
	for k in range(0,11):
		d += distance()
	d=round(d/10,2)
	return d

if __name__ == "__main__":
    t1 = time.time()
    startNode = [270,30,0]
    goalNode = [45,45]
    searchPos = [[90,90,10],[100,100,345],[210,100,345]] # Chnage these
    searchAngles = [335,0,25]
    p,b,s = 0,0,0
    #Initialize Serial Port and PWM Pins
    print('\n-------->Program Start<--------')
    print('\n-------->Initializing Serial and GPIO pins<--------')
    init()
    print('\n-------->Current Parameters<--------')
    startNode[2] = getYaw()
    print('\nCurrent Position: ',startNode)
    newPos = startNode
    print('\n-------->Script Start<--------')
    color ={0:'Red',1:'Green',2:'Blue'}
    mask = [0,1,2,0,1,2,0,1,2]
    blockGrab = 0
    txtfilename = 'positiondata_'+str(time.time())+'.txt'
    f = open(txtfilename,'a')
    try: 
        while True:
            ti_1 = time.time()
            ret,frame = get_frame()
            if ret==True:
                #newPos = calcNewPos(newPos)
                data = str(newPos)+'\n'
                f.write(data)
                h,center = 0,0
                get_frame()
                s = go2SearchAngle(s)
                if s == 3:
                    s = 0
                    p +=1
                    if p == len(searchPos)-1: p = 1 
                    newPos = go2Pos(newPos,searchPos[p]) 
                print('Searching for object....')
                continue
            else:
                print('Camera Fail')
                break
        ti2 = time.time()
        print('\nTime for each iteratiion', ti2-ti_1)
            
    except KeyboardInterrupt:
        print('\n-------->Interrupt detected<--------')
    finally:
        t2 = time.time()
        print('\nRuntime: ',t2-t1,' seconds')
        print('\n-------->Cleaning up pins<--------')
        gameover()
        gpio.cleanup()
        cv2.destroyAllWindows()
        f.close()
        print('\n-------->Cleaning up successful<--------')
        print('\n-------->Program End<--------')