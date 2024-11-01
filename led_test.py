#LED stuff
import RPi.GPIO as GPIO			# Import the GPIO module
import time 				# Import the time module
GPIO.setmode(GPIO.BCM)			# Set the GPIO pin naming convention to BCM
GPIO.setup(13,GPIO.OUT)			# Set up GPIO pin 21 as an output
GPIO.setup(19,GPIO.OUT)			# Set up GPIO pin 21 as an output
GPIO.setup(26,GPIO.OUT)			# Set up GPIO pin 21 as an output


def LED(colour, on_off):
    if(colour == "red"):
        if(on_off == "on"):
            GPIO.output(13,GPIO.HIGH) 		# Set GPIO pin 20 to digital high (on)
        elif(on_off == "off"):
            GPIO.output(13,GPIO.LOW)		# Set GPIO pin 20 to digital low (off)
    elif(colour == "yellow"):
        if(on_off == "on"):
            GPIO.output(19,GPIO.HIGH) 		# Set GPIO pin 20 to digital high (on)
        elif(on_off == "off"):
            GPIO.output(19,GPIO.LOW)		# Set GPIO pin 20 to digital low (off)
    elif(colour == "green"):
        if(on_off == "on"):
            GPIO.output(26,GPIO.HIGH) 		# Set GPIO pin 20 to digital high (on)
        elif(on_off == "off"):
            GPIO.output(26,GPIO.LOW)		# Set GPIO pin 20 to digital low (off)


# ###debug red-yellow-green cycle
# while(True):
#     LED("red", "on")
#     time.sleep(0.5)
#     LED("yellow", "on")
#     time.sleep(0.5)
#     LED("green", "on")
#     time.sleep(0.5)
#     LED("red", "off")
#     time.sleep(0.5)
#     LED("yellow", "off")
#     time.sleep(0.5)
#     LED("green", "off")
#     time.sleep(0.5)