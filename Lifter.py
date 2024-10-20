from gpiozero import AngularServo
from time import sleep

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
            self.smooth_move(self.servo, self.servo.angle, 17)
            self.current_angle = 17  # Update current angle
        elif height == 1:
            print("Moving to middle shelf")
            self.smooth_move(self.servo, self.servo.angle, 55)
            self.current_angle = 55  # Update current angle
        elif height == 2:
            print("Moving to top shelf")
            self.smooth_move(self.servo, self.servo.angle, 90)
            self.current_angle = 90  # Update current angle
        else:
            print("Invalid choice, please enter 0, 1, or 2.")
        
        # Detach to stop sending PWM signals after reaching the target position (stops constant servo jitter)
        sleep(1)
        self.servo.detach()

    # Helper function to move the servo smoothly to the target angle
    def smooth_move(self, servo, start_angle, target_angle, steps=100, delay=0.02):
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