import RPi.GPIO as GPIO
import time

class grab:
    pwm = None

    def __init__(self, servo_pin=16):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(servo_pin, GPIO.OUT)
        
        # Set up PWM at 50Hz (standard for servos)
        self.pwm = GPIO.PWM(servo_pin, 50)
        self.pwm.start(0)

    # Function to move the servo to a specific angle
    def Set_servo_angle(self, angle):
        if angle < 10:
            angle = 10
        elif angle > 180:
            angle = 180

        duty_cycle = 2 + (angle / 18)  # Convert angle to duty cycle for 180-degree servo
        print(f"Setting angle to {angle} degrees, duty cycle: {duty_cycle}")
        
        self.pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.5)
        self.pwm.ChangeDutyCycle(0)  # Stop PWM after moving

    # Function to perform the grab action
    def Grab(self, choice):
        if choice == 1:
            print("Moving to grab position (115 degrees)")
            self.Set_servo_angle(130)
        elif choice == 2:
            print("Moving to release posdef choose 1 (grab) or 2 (release).")
            self.Set_servo_angle(10)

    # Cleanup function to stop PWM and reset GPIO
    def Dispose(self):
        self.pwm.stop()
        GPIO.cleanup()

# If this script is run directly, test grab functionality
if __name__ == "__main__":
    grabber = grab()
    try:
        choice = int(input("Enter 1 to grab, 2 to release: "))
        grabber.Grab(choice)  # Call grab with user choice
    finally:
        grabber.Dispose()  # Ensure cleanup after execution
