from gpiozero import AngularServo
from time import sleep

# Helper function to move the servo smoothly to the target angle
# SMOOOOOOOTH OPERATORRRRRRRR
# Helper function to move the servo smoothly to the target angle
# Helper function to move the servo smoothly to the target angle
def smooth_move(servo, start_angle, target_angle, steps=100, delay=0.02):
    # Ensure start_angle is valid; if None, use current angle or sensible default
    if start_angle is None:
        start_angle = target_angle  # Set start angle to target if undefined
    
    # Calculate the difference and step size
    angle_diff = target_angle - start_angle
    step_size = angle_diff / steps

    # Move incrementally towards the target
    for i in range(steps):
        current_angle = start_angle + (i * step_size)
        servo.angle = current_angle  # Update the servo angle incrementally
        sleep(delay)  # Small delay between steps to create smooth movement

    # Final move to the exact target angle
    servo.angle = target_angle
    sleep(delay)

# Lifter class
# Lifter class
class Lifter:
    def __init__(self, servo_pin=25):
        self.servo = AngularServo(servo_pin, min_angle=180, max_angle=0, min_pulse_width=0.0005, max_pulse_width=0.0025)
        self.current_angle = 90  # Start from a default known angle
        # Detach the servo immediately after initialization to avoid jitter
        sleep(1)
        self.servo.detach()

    def set_height(self, height):
        if self.servo.angle is None:
            # Initialize servo angle to the last known position (or default)
            self.servo.angle = self.current_angle

        if height == 0:
            print("Moving to bottom shelf")
            smooth_move(self.servo, self.servo.angle, 17)
            self.current_angle = 17  # Update current angle
        elif height == 1:
            print("Moving to middle shelf")
            smooth_move(self.servo, self.servo.angle, 60)
            self.current_angle = 55  # Update current angle
        elif height == 2:
            print("Moving to top shelf")
            smooth_move(self.servo, self.servo.angle, 90)
            self.current_angle = 90  # Update current angle
        else:
            print("Invalid choice, please enter 0, 1, or 2.")
        
        # Detach to stop sending PWM signals after reaching the target position (stops constant servo jitter)
        sleep(1)
        self.servo.detach()

# Grabber class
class Grabber:
    def __init__(self, servo_pin=16):
        self.servo = AngularServo(servo_pin, min_angle=0, max_angle=180, min_pulse_width=0.0005, max_pulse_width=0.0025)
        self.current_angle = 15  # Default angle to open
        # Detach the servo immediately after initialization to avoid jitter
        sleep(1)
        self.servo.detach()

    def grab(self, position):
        # Ensure the servo is attached before movement
        self.servo.angle = self.current_angle

        if position == 0:
            print("Releasing")
            smooth_move(self.servo, self.current_angle, 15, steps=100, delay=0.02)  # Adjusted steps and delay for smooth release
            self.current_angle = 15  # Update current angle
            sleep(1)
            self.servo.detach()  # Detach after releasing to stop signals
        elif position == 1:
            print("Grabbing")
            smooth_move(self.servo, self.current_angle, 118, steps=100, delay=0.02)  # Adjusted steps and delay for smooth grab
            self.current_angle = 118  # Update current angle
            sleep(1)
            #self.servo.detach()  # Detach after grabbing to stop signals

        else:
            print("Invalid choice, please enter 0 for release, or 1 for grab.")

# Function to handle user input and control servos
def control_servos():
    lifter = Lifter()
    grabber = Grabber()

    while True:
        # Prompt the user to select the servo
        servo_choice = input("Select servo (lifter/grabber): ").lower()

        if servo_choice == "lifter":
            # Prompt for the height
            try:
                height = int(input("Enter height (0 for bottom, 1 for middle, 2 for top): "))
                lifter.set_height(height)
            except ValueError:
                print("Invalid input. Please enter a number (0, 1, or 2).")

        elif servo_choice == "grabber":
            # Prompt for grab/release
            try:
                position = int(input("Enter position (0 for release, 1 for grab): "))
                grabber.grab(position)
            except ValueError:
                print("Invalid input. Please enter 0 for release, or 1 for grab.")

        else:
            print("Invalid choice, please select either 'lifter' or 'grabber'.")

if __name__ == "__main__":
    control_servos()
