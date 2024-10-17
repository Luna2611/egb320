from gpiozero import AngularServo
from time import sleep

# Set up the AngularServo (using GPIO pin 25 as in your code)
servo = AngularServo(25, min_angle=180, max_angle=0, min_pulse_width=0.0005, max_pulse_width=0.0025)


### grab.py initi

import RPi.GPIO as GPIO
import time

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Choose the GPIO pin to control the servo 
servo_pin = 16

# Set up the GPIO pin for PWM at 50Hz (servo standard frequency)
GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, 50)  # 50Hz PWM frequencyå

# Start PWM running, but with the servo at 0 degrees (duty cycle = 0)
pwm.start(0)

# Function to move the servo to a specific angle for a 270-degree servo
def set_servo_angle(angle):
    # Ensure the angle is within the servo's physical limits (0 to 180 degrees)
    if angle < 10:
        angle = 10
    elif angle > 180:
        angle = 180

    # Convert the angle to the duty cycle for a 180-degree servo
    # Calibrating: Adjust the range based on the servo's specification (typically 0.5ms - 2.5ms pulse width)
    duty_cycle = 2 + (angle / 18)  # Adjust for a 180-degree servo
    print(f"Setting angle to {angle} degrees, duty cycle: {duty_cycle}")
    
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    GPIO.output(servo_pin, False)
    #pwm.ChangeDutyCycle(0)

def shelfHeight(height):

        if height == 0:
                print("Moving to bottom shelf")
                servo.angle = 15  # Bottom shelf

        elif height == 1:
                print("Moving to middle shelf")
                servo.angle = 40  # Middle shelf

        elif height == 2:
                print("Moving to top shelf")
                servo.angle = 70  # Top shelf

        else:
                print("Invalid choice, please enter 1, 2, or 3.")

        # Allow time for the servo to move
        sleep(1)  # Adjust sleep duration as per your servo's speed

        # Stop sending commands to minimize jiggle
        servo.detach()  # Stops sending further commands to the servo

def grab(choice):
        #choice = input("Enter 1 to grab, 2 to release: ")

        if choice == 1:
            print("Moving to 180 degrees")
            set_servo_angle(115)
        elif choice == 1:
            print("Moving to 0 degrees")
            set_servo_angle(18)
            pwm.ChangeDutyCycle(0)
        else:
            print("Invalid choice. Please choose 1, 2, or 3.")

grab(2)
shelfHeight(2)