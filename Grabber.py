from gpiozero import AngularServo
from time import sleep

class Grabber:
    def __init__(self, servo_pin=16):
        self.servo = AngularServo(servo_pin, min_angle=0, max_angle=180, min_pulse_width=0.0005, max_pulse_width=0.0025)
        self.current_angle = 15  # Default angle to open
        # Detach the servo immediately after initialization to avoid jitter
        sleep(1)
        self.servo.detach()

    def Grab(self, position):
        # Ensure the servo is attached before movement
        self.servo.angle = self.current_angle

        if position == 0:
            print("Releasing")
            self.smooth_move(self.servo, self.current_angle, 15, steps=100, delay=0.02)  # Adjusted steps and delay for smooth release
            self.current_angle = 15  # Update current angle
            sleep(1)
            self.servo.detach()  # Detach after releasing to stop signals
        elif position == 1:
            print("Grabbing")
            self.smooth_move(self.servo, self.current_angle, 118, steps=100, delay=0.02)  # Adjusted steps and delay for smooth grab
            self.current_angle = 118  # Update current angle
            sleep(1)
            #self.servo.detach()  # Detach after grabbing to stop signals

        else:
            print("Invalid choice, please enter 0 for release, or 1 for grab.")

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
