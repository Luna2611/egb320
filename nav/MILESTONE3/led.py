#LED stuff
import RPi.GPIO as GPIO			# Import the GPIO module
import time 				# Import the time module

GPIO_PIN_GREEN = 21
GPIO_PIN_YELLOW = 6
GPIO_PIN_RED = 5

class LED:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)			# Set the GPIO pin naming convention to BCM
        GPIO.setup(GPIO_PIN_GREEN,GPIO.OUT)			# Set up GPIO pin 21 as an output
        GPIO.setup(GPIO_PIN_YELLOW,GPIO.OUT)			# Set up GPIO pin 21 as an output
        GPIO.setup(GPIO_PIN_RED,GPIO.OUT)			# Set up GPIO pin 21 as an output

    def toggle(self, colour, on_off):      
        colour = colour.lower()
        on_off = on_off.lower()
        if(colour == "all"):
            if(on_off == "on"):
                GPIO.output(GPIO_PIN_GREEN,GPIO.HIGH)
                GPIO.output(GPIO_PIN_YELLOW,GPIO.HIGH)
                GPIO.output(GPIO_PIN_RED,GPIO.HIGH)  		# Set GPIO pin 20 to digital high (on)
            elif(on_off == "off"):
                GPIO.output(GPIO_PIN_GREEN,GPIO.LOW)
                GPIO.output(GPIO_PIN_YELLOW,GPIO.LOW) 		# Set GPIO pin 20 to digital low (off)
                GPIO.output(GPIO_PIN_RED,GPIO.LOW)
        elif(colour == "red"):
            if(on_off == "on"):
                GPIO.output(GPIO_PIN_RED,GPIO.HIGH) 		# Set GPIO pin 20 to digital high (on)
            elif(on_off == "off"):
                GPIO.output(GPIO_PIN_RED,GPIO.LOW)		# Set GPIO pin 20 to digital low (off)
        elif(colour == "yellow"):
            if(on_off == "on"):
                GPIO.output(GPIO_PIN_YELLOW,GPIO.HIGH) 		# Set GPIO pin 20 to digital high (on)
            elif(on_off == "off"):
                GPIO.output(GPIO_PIN_YELLOW,GPIO.LOW)		# Set GPIO pin 20 to digital low (off)
        elif(colour == "green"):
            if(on_off == "on"):
                GPIO.output(GPIO_PIN_GREEN,GPIO.HIGH) 		# Set GPIO pin 20 to digital high (on)
            elif(on_off == "off"):
                GPIO.output(GPIO_PIN_GREEN,GPIO.LOW)		# Set GPIO pin 20 to digital low (off)

    def Dispose(self):
        GPIO.cleanup()

    # GPIO.setup(21,GPIO.OUT)			# Set up GPIO pin 21 as an output
    # GPIO.output(21,GPIO.HIGH) 		# Set GPIO pin 21 to digital high (on)
    # GPIO.setup(20,GPIO.OUT)			# Set up GPIO pin 21 as an output
    # GPIO.output(20,GPIO.HIGH) 		# Set GPIO pin 21 to digital high (on)
    # time.sleep(5)				# Wait for 5 seconds
    # GPIO.output(21,GPIO.LOW)		# Set GPIO pin 21 to digital low (off)
    # GPIO.output(20,GPIO.LOW)		# Set GPIO pin 21 to digital low (off)
    # GPIO.cleanup()				# Exit the GPIO session cleanly

        # print("stuff")
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
        #     LED("all", "on")
        #     time.sleep(0.5)
        #     LED("all", "off")
        #     time.sleep(0.5)
   

