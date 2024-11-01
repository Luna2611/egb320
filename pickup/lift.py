'''
import RPi.GPIO as GPIO
import time

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Choose the GPIO pin to control the servo 
servo_pin = 25

# Set up the GPIO pin for PWM at 50Hz (servo standard frequency)
GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, 50)  # 50Hz PWM frequency

# Start PWM running, but with the servo at 0 degrees (duty cycle = 0)
pwm.start(0)

# Function to move the servo to a specific angle for a 270-degree servo
def set_servo_angle(angle, time_length):
    # Ensure the angle is within the servo's physical limits (0 to 270 degrees)
    if angle < 0:
        angle = 0
    elif angle > 270:
        angle = 270

    # Convert the angle to the duty cycle for a 270-degree servo
    # Calibrating: Adjust the range based on the servo's specification (typically 0.5ms - 2.5ms pulse width)
    duty_cycle = 1.3 + (angle / 100)  # Adjust for a 270-degree servo
    print(f"Setting angle to {angle} degrees, duty cycle: {duty_cycle}")
    
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(time_length)
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

def set_home():
    set_servo_angle(0,2)
    


try:
    while True:
        # Ask the user to choose an angle
        print("enter duty cycle")


	

        choice = input("enter duty cycle: ")

        if choice == '1':
            print("Moving to 0 degrees")
            set_servo_angle(-30, 1)
        elif choice == '2':
            print("Moving to 45 degrees")
            set_servo_angle(10,1)
        elif choice == '3':
            print("Moving to 180 degrees")
            set_servo_angle(20,1)
        elif choice == '4':
            print("Moving to 45 degrees")
            set_servo_angle(30,1)
        elif choice == '5':
            print("Moving to 180 degrees")
            set_servo_angle(30,1)
        elif choice == '6':
            print("Moving to 45 degrees")
            set_servo_angle(40,1)
        elif choice == '7':
            print("Moving to 180 degrees")
            set_servo_angle(70,1)
        else:
            print("Invalid choice")
        
        time.sleep(2)


# Function to move the servo based on a specified duty cycle
def set_servo_duty_cycle(duty_cycle, time_length):
    print(f"Setting duty cycle to {duty_cycle}")
    
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(time_length)
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

# Function to return the servo to home position using a specific duty cycle (0 degrees position)
def set_home():
    set_servo_duty_cycle(2, 2)  # Duty cycle for 0 degrees (home position), adjust if needed

try:
    while True:
        # Ask the user to enter a duty cycle
        duty_cycle = float(input("Enter duty cycle (1 to 2 for standard servo control): "))
        #time_length = float(input("Enter the time to hold position (in seconds): "))
        time_length = 0.5
        
        # Ensure the duty cycle is within a safe range for the servo
        if duty_cycle == 1000:
            print("Invalid duty cycle! Enter a value between 2 and 12.")
        else:
            set_servo_duty_cycle(duty_cycle, time_length)



except KeyboardInterrupt:
    print("Program stopped by user")

finally:
    pwm.stop()
    GPIO.cleanup()

import RPi.GPIO as GPIO
from gpiozero import AngularServo
from time import sleep
servo_pin = 25

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Choose the GPIO pin to control the servo 
servo_pin = 16

# Set up the GPIO pin for PWM at 50Hz (servo standard frequency)
GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, 50)  # 50Hz PWM frequency


servo = AngularServo(25, min_angle=180, max_angle=0, min_pulse_width=0.0005, max_pulse_width=0.0025)

try:
    while True:
        # Ask the user to choose an angle
        choice = input("Enter shelf level (1, 2, or 3): ")

        if choice == '1':
            print("Moving to bottom shelf")
            servo.angle = 30  #bottom shelf

        elif choice == '2':
            print("Moving to middle shelf")
            servo.angle = 90  # Middle shelf

        elif choice == '3':
            print("Moving to top shelf")
            servo.angle = 160  # Top shelf


        else:
            print("Invalid choice, please enter 1, 2, or 3.")

        sleep(1)  # Allow time for the servo to move
        pwm.ChangeDutyCycle(0)


except KeyboardInterrupt:
    print("Program terminated.")
'''  
from gpiozero import AngularServo
from time import sleep

# Set up the AngularServo (using GPIO pin 25 as in your code)
servo = AngularServo(25, min_angle=180, max_angle=0, min_pulse_width=0.0005, max_pulse_width=0.0025)

try:
    while True:
        # Ask the user to choose an angle
        choice = input("Enter shelf level (1, 2, or 3): ")

        if choice == '1':
            print("Moving to bottom shelf")
            servo.angle = 15  # Bottom shelf

        elif choice == '2':
            print("Moving to middle shelf")
            servo.angle = 40  # Middle shelf

        elif choice == '3':
            print("Moving to top shelf")
            servo.angle = 70  # Top shelf

        else:
            print("Invalid choice, please enter 1, 2, or 3.")
            continue

        # Allow time for the servo to move
        sleep(1)  # Adjust sleep duration as per your servo's speed

        # Stop sending commands to minimize jiggle
        servo.detach()  # Stops sending further commands to the servo

except KeyboardInterrupt:
    print("Program terminated.")

'''

from gpiozero import AngularServo
from time import sleep

# Set up the AngularServo (using GPIO pin 25 as in your code)
servo = AngularServo(25, min_angle=180, max_angle=0, min_pulse_width=0.0005, max_pulse_width=0.0025)

# Function to gradually move the servo to the target angle
def move_servo_slowly(servo, target_angle, step=1, delay=0.05):
    current_angle = servo.angle
    
    # If the angle is None (servo was detached), assume it's at the target angle
    if current_angle is None:
        current_angle = target_angle  # Set directly to the target angle
    
    # Move the servo incrementally in small steps
    while abs(current_angle - target_angle) > abs(step):
        if current_angle < target_angle:
            current_angle += step
        else:
            current_angle -= step
        servo.angle = current_angle
        sleep(delay)
    
    # Ensure the final angle is exactly the target angle
    servo.angle = target_angle

try:
    while True:
        # Ask the user to choose an angle
        choice = input("Enter shelf level (1, 2, or 3): ")

        if choice == '1':
            print("Moving to bottom shelf")
            move_servo_slowly(servo, 30, step=1, delay=0.05)  # Move slowly to bottom shelf

        elif choice == '2':
            print("Moving to middle shelf")
            move_servo_slowly(servo, 90, step=1, delay=0.05)  # Move slowly to middle shelf

        elif choice == '3':
            print("Moving to top shelf")
            move_servo_slowly(servo, 160, step=1, delay=0.05)  # Move slowly to top shelf

        else:
            print("Invalid choice, please enter 1, 2, or 3.")
            continue

        # Allow time for the servo to settle
        sleep(1)

        # Detach the servo after the move is complete to prevent jitter
        servo.detach()

except KeyboardInterrupt:
    print("Program terminated.")

'''