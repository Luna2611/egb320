from gpiozero import AngularServo
from time import sleep

class Grabber:
    servo = None

    def __init__(self):
        self.servo = AngularServo(16, min_angle=0, max_angle=180, min_pulse_width=0.0005, max_pulse_width=0.0025)
    
    def Grab(self, position):
        if position == 0:
            print("releasing")
            self.servo.angle = 15  #release 
            sleep(1)
            self.servo.detach()

        elif position == 1:
            print("grabbing")
            self.servo.angle = 118  #grab

        else:
            print("Invalid choice")
