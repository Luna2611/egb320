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

try:
    while True:
        # Ask the user to choose an angle

        choice = input("Enter 1 to grab, 2 to release: ")

        if choice == '1':
            print("Moving to 180 degrees")
            set_servo_angle(115)
        elif choice == '2':
            print("Moving to 0 degrees")
            set_servo_angle(18)
            pwm.ChangeDutyCycle(0)
        else:
            print("Invalid choice. Please choose 1, 2, or 3.")
        
        #time.sleep(2)

except KeyboardInterrupt:
    print("Program stopped by user")

finally:
    pwm.stop()
    GPIO.cleanup()
