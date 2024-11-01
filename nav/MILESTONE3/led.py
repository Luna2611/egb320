#LED stuff
import RPi.GPIO as GPIO			# Import the GPIO module
import time 				# Import the time module

GPIO_PIN_GREEN = 26
GPIO_PIN_RED = 19
GPIO_PIN_BLUE = 13

class LED:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)			# Set the GPIO pin naming convention to BCM
        GPIO.setup(GPIO_PIN_GREEN,GPIO.OUT)			# Set up GPIO pin 21 as an output
        GPIO.setup(GPIO_PIN_BLUE,GPIO.OUT)			# Set up GPIO pin 21 as an output
        GPIO.setup(GPIO_PIN_RED,GPIO.OUT)			# Set up GPIO pin 21 as an output

    def toggle(self, colour, on_off):      
        colour = colour.lower()
        on_off = on_off.lower()
        if(colour == "all"):
            if(on_off == "on"):
                GPIO.output(GPIO_PIN_GREEN,GPIO.HIGH)
                GPIO.output(GPIO_PIN_BLUE,GPIO.HIGH)
                GPIO.output(GPIO_PIN_RED,GPIO.HIGH)
            elif(on_off == "off"):
                GPIO.output(GPIO_PIN_GREEN,GPIO.LOW)
                GPIO.output(GPIO_PIN_BLUE,GPIO.LOW)
                GPIO.output(GPIO_PIN_RED,GPIO.LOW)
        elif(colour == "red"):
            if(on_off == "on"):
                self.toggle("all", "off")
                GPIO.output(GPIO_PIN_RED,GPIO.HIGH)
            elif(on_off == "off"):
                GPIO.output(GPIO_PIN_RED,GPIO.LOW)
        elif(colour == "yellow"):
            if(on_off == "on"):
                self.toggle("all", "off")
                GPIO.output(GPIO_PIN_GREEN,GPIO.HIGH)
                GPIO.output(GPIO_PIN_RED,GPIO.HIGH)
            elif(on_off == "off"):
                GPIO.output(GPIO_PIN_GREEN,GPIO.LOW)
                GPIO.output(GPIO_PIN_RED,GPIO.LOW)
        elif(colour == "green"):
            if(on_off == "on"):
                self.toggle("all", "off")
                GPIO.output(GPIO_PIN_GREEN,GPIO.HIGH)
            elif(on_off == "off"):
                GPIO.output(GPIO_PIN_GREEN,GPIO.LOW)
        elif(colour == "blue"):
            if(on_off == "on"):
                self.toggle("all", "off")
                GPIO.output(GPIO_PIN_BLUE,GPIO.HIGH)
            elif(on_off == "off"):
                GPIO.output(GPIO_PIN_BLUE,GPIO.LOW)	

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

