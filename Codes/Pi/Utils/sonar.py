import RPi.GPIO as gpio
import time

gpio.setwarnings(False)

trig = 16
echo = 18

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

	