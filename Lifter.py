from gpiozero import AngularServo
from time import sleep

class Lifter:
    servo = None
    def __init__(self, servo_pin=25):
        # Set up the AngularServo (using GPIO pin 25 as in your code)
        self.servo = AngularServo(servo_pin, min_angle=180, max_angle=0, min_pulse_width=0.0005, max_pulse_width=0.0025)

    def set_height(self, height, sleep_period=1):
        """
        Moves the servo to the specified shelf height.
        
        :param height: 0 for bottom shelf, 1 for middle shelf, 2 for top shelf
        """
        if height == 0:
            print("Moving to bottom shelf")
            self.servo.angle = 15  # Bottom shelf

        elif height == 1:
            print("Moving to middle shelf")
            self.servo.angle = 40  # Middle shelf

        elif height == 2:
            print("Moving to top shelf")
            self.servo.angle = 90  # Top shelf

        else:
            print("Invalid choice, please enter 0, 1, or 2.")

        # Allow time for the servo to move
        sleep(sleep_period)  # Adjust sleep duration as per your servo's speed

        # Stop sending commands to minimize jiggle
        self.servo.detach()  # Stops sending further commands to the servo
