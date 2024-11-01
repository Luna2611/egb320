from __future__ import print_function

import numpy as np
import cv2
import sys
from vision import Vision
# from item_collection import grab
# from item_collection import shelfHeight


# Item management
from Grabber import Grabber  # Import the grab module

from Lifter import Lifter 

# Set up the GPIO and PWM
# pwm = Grab.setup_gpio()

# Perform a grab action (choice = 1 for grabbing, 2 for releasing)
# grab.grab(pwm, 1)  # Example grab

# # Perform a release action
# grab.grab(pwm, 2)  # Example release

# # Clean up GPIO when done
# grab.cleanup_gpio(pwm)


# # main.py

# # Call the shelfHeight function to move to the desired shelf
# # shelfHeight(0)  # Move to bottom shelf
# # shelfHeight(1)  # Move to middle shelf
# shelfHeight(1)  # Move to top shelf
# shelfHeight(2)
# shelfHeight(2)
# shelfHeight(2)
# shelfHeight(2)
# shelfHeight(2)
# shelfHeight(2)


from nav.MILESTONE3 import LED

from mobility import DRIVE_FUNCTION
from mobility import THIS_BOARD_TYPE, DFRobot_DC_Motor_IIC as Board

### Motor INitialisation


import os
#sys.path.append("../")

import time



if THIS_BOARD_TYPE:
  board = Board(1, 0x10)    # RaspberryPi select bus 1, set address to 0x10
else:
  board = Board(7, 0x10)    # RockPi select bus 7, set address to 0x10

def board_detect():
  l = board.detecte()
  print("Board list conform:")
  print(l)

''' print last operate status, users can use this variable to determine the result of a function call. '''
def print_board_status():
  if board.last_operate_status == board.STA_OK:
    print("board status: everything ok")
  elif board.last_operate_status == board.STA_ERR:
    print("board status: unexpected error")
  elif board.last_operate_status == board.STA_ERR_DEVICE_NOT_DETECTED:
    print("board status: device not detected")
  elif board.last_operate_status == board.STA_ERR_PARAMETER:
    print("board status: parameter error, last operate no effective")
  elif board.last_operate_status == board.STA_ERR_SOFT_VERSION:
    print("board status: unsupport board framware version")

if __name__ == "__main__":

  led = LED()

  board_detect()    # If you forget address you had set, use this to detected them, must have class instance

  # Set board controler address, use it carefully, reboot module to make it effective
  '''
  board.set_addr(0x10)
  if board.last_operate_status != board.STA_OK:
    print("set board address faild")
  else:
    print("set board address success")
  '''

  while board.begin() != board.STA_OK:    # Board begin and check board status
    print_board_status()
    print("board begin failed")
    time.sleep(2)
  print("board begin success")

  board.set_encoder_enable(board.ALL)                 # Set selected DC motor encoder enable
  # board.set_encoder_disable(board.ALL)              # Set selected DC motor encoder disable
  board.set_encoder_reduction_ratio(board.ALL, 35)    # Set selected DC motor encoder reduction ratio, test motor reduction ratio is 43.8

  board.set_moter_pwm_frequency(1000)   # Set DC motor pwm frequency to 1000HZ

###end motor initialisation
###constants
duty_cycle = 80
turning_duty_cycle = 100
slow_turn_speed = 70
state = "openloopturningtoshelf"
if __name__ == "__main__":   
    grabber = Grabber()
    lifter = Lifter()
    vision = Vision()
    vision.SetupCamera()
    lifter.set_height(1)
    try:
        while(1):
            lifter.set_height(0)
            itemsBearing, obstaclesRangeBearing, packingBayBearing, bayMarkerRangeBearing, rowMarkersRangeBearing, shelfBearing, wallRange = vision.Run()

            print("\n\n")

            if len(itemsBearing) > 0:
                print( "Amount of items: " + str(len(itemsBearing)))
                for item in itemsBearing:
                    print("Item Bearing: " + str(item))
            
            if len(obstaclesRangeBearing) > 0:
                print( "Amount of obstacles:" + str(len(obstaclesRangeBearing)) )
                for obstacle in obstaclesRangeBearing:
                    print("Obstacle Bearing: " + str(obstacle[0]))
                    print("Obstacle Distance: " + str(obstacle[1]))

            
                print("Packing Bay Bearing: " + str(packingBayBearing))

            if len(bayMarkerRangeBearing) > 0:
                print("Packing Bay Marker Range: " + str(bayMarkerRangeBearing[0]))
                print("Packing Bay Marker Bearing: " + str(bayMarkerRangeBearing[1]))
                led.toggle("yellow", "on")

            #if len(rowMarkersRangeBearing) > 0:
            #print("cant see shit")
            seen_rowmarker = None
            board.motor_stop(board.ALL)   # stop all DC motor
            led.toggle("all", "off")
            for rowMarker in rowMarkersRangeBearing:
                if rowMarker:
                    # print("This is row: " + str(rowMarker[0]))
                    # print("Row Marker Range: " + str(rowMarker[1]))
                    # print("Row Marker Bearing: " + str(rowMarker[2]))
                    seen_rowmarker = rowMarker

            if len(shelfBearing) > 0:
                #print( "Amount of shelves: " + str(len(shelfBearing)))
                i = 0
                for shelf in shelfBearing:
                    #print("Shelf Bearing: " + str(shelfBearing[i]))
                    i+=1

            speed = board.get_encoder_speed(board.ALL)      # Use boadrd.all to get all encoders speed
            print(speed[0], speed[1])
            #### speed[0] is the only one that gives good values at the moment - LEFT HAND MOTOR SPEED GOOD





            #### BEGIN STATE MACHINE
            print(state)
            if(state == "parked"):
                # grabber.Grab(0)
                # time.sleep(1)
                # grabber.Grab(1)
                # time.sleep(2)
                time.sleep(0.1)

            elif(state == "drivingoutrow"):
                lifter.set_height(0)
                if(seen_rowmarker and seen_rowmarker[1] > 700):
                    state = "parked"
                if(seen_rowmarker and seen_rowmarker[1] < 500):
                    targetBearing = seen_rowmarker[2]
                    # if(1):
                else:
                    targetBearing = None
                    #turn on spot right
                    print("blind reversing")
                    board.motor_movement([board.M1], board.CW, slow_turn_speed)
                    board.motor_movement([board.M2], board.CCW, slow_turn_speed)

                if(targetBearing is not None):
                    if(abs(targetBearing)<15):
                        #drive straight
                        print("drive straight")
                        board.motor_movement([board.M1], board.CW, duty_cycle-targetBearing)
                        board.motor_movement([board.M2], board.CCW, duty_cycle+targetBearing)
                    elif(targetBearing>0):
                        print("shuffle right")
                        board.motor_movement([board.M1], board.CW, duty_cycle/2)
                        board.motor_movement([board.M2], board.CW, duty_cycle)
                    else:
                        print("shuffle left")
                        board.motor_movement([board.M1], board.CCW, duty_cycle)
                        board.motor_movement([board.M2], board.CCW, duty_cycle/2)

            elif(state == "pickupitem"):
                time.sleep(1)
                board.motor_movement([board.M1], board.CCW, duty_cycle)
                board.motor_movement([board.M2], board.CW, duty_cycle)
                time.sleep(0.35)
                board.motor_stop(board.ALL)   # stop all DC motor
                grabber.Grab(1)
                time.sleep(1)
                board.motor_movement([board.M1], board.CW, duty_cycle)
                board.motor_movement([board.M2], board.CCW, duty_cycle)
                time.sleep(0.35)
                board.motor_movement([board.M1], board.CCW, 100)
                board.motor_movement([board.M2], board.CCW, 100)
                time.sleep(0.6)
                board.motor_movement([board.M1], board.CW, duty_cycle)
                board.motor_movement([board.M2], board.CCW, duty_cycle)
                time.sleep(1)
                # board.motor_stop(board.ALL)   # stop all DC motor
                

                state = "drivingoutrow"

            elif(state == "openloopturningtoshelf"):
                # print("GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG")
                # grab(1)
                # time.sleep(10)
                # grab(2)


                # print(str(wallRange))
                # time.sleep(1)
                board.motor_movement([board.M1], board.CW, 100)
                board.motor_movement([board.M2], board.CW, 100)
                time.sleep(0.6)
                board.motor_stop(board.ALL)   # stop all DC motor

                state = "turntowardsitem"



            elif(state == "turntowardsitem"):
              if(itemsBearing):
                  itemsBearing.sort() #key=abs
                  # print("QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ")
                  # desired_angle = itemsBearing[0]
                  if(abs(itemsBearing[0])<10):
                    print("THE FUCK???")
                    state = "pickupitem"
                    #time.sleep(0.1)
                  elif(itemsBearing[0]>0):
                    print("turn left")
                    board.motor_movement([board.M1], board.CCW, slow_turn_speed)
                    board.motor_movement([board.M2], board.CCW, slow_turn_speed)
                  else:
                    print("turn right")
                    board.motor_movement([board.M1], board.CW, slow_turn_speed)
                    board.motor_movement([board.M2], board.CW, slow_turn_speed)
              else:
                  print("turn left")
                  board.motor_movement([board.M1], board.CW, slow_turn_speed)
                  board.motor_movement([board.M2], board.CW, slow_turn_speed)
            elif(state == "drivingdownrow"):
                #print(len(shelfBearing))
                if(seen_rowmarker and seen_rowmarker[1] < 1800):

                                    # if(seen_rowmarker[1] < 1600): # third bay is 450 second bay is 800 first bay is 1150

                    state = "turningtoshelf"
                if(seen_rowmarker and seen_rowmarker[1] < 1200):
                    targetBearing = seen_rowmarker[2]
                    # if(1):
                elif(len(shelfBearing) == 2):
                    targetBearing = (shelfBearing[0] + shelfBearing[1]) / 2
                    print("YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY")
                else:
                    targetBearing = None
                    #turn on spot right
                    print("turn on spot right")
                    board.motor_movement([board.M1], board.CCW, turning_duty_cycle)
                    board.motor_movement([board.M2], board.CCW, turning_duty_cycle)

                if(targetBearing is not None):
                    if(abs(targetBearing)<15):
                        #drive straight
                        print("drive straight")
                        board.motor_movement([board.M1], board.CCW, duty_cycle+targetBearing)
                        board.motor_movement([board.M2], board.CW, duty_cycle-targetBearing)
                    elif(targetBearing>0):
                        print("shuffle right")
                        board.motor_movement([board.M1], board.CCW, duty_cycle)
                        board.motor_movement([board.M2], board.CCW, duty_cycle/2)
                    else:
                        print("shuffle left")
                        board.motor_movement([board.M1], board.CW, duty_cycle/2)
                        board.motor_movement([board.M2], board.CW, duty_cycle)
            elif(state == "liningupwithrow"):
                #turn on spot left
                print("turn on spot left")
                board.motor_movement([board.M1], board.CW, turning_duty_cycle)
                board.motor_movement([board.M2], board.CW, turning_duty_cycle)                
                if(packingBayBearing):
                    if(packingBayBearing < 1500):
                      state = "drivingdownrow"
                    board.motor_stop(board.ALL)   # stop all DC motor
                    if(abs(packingBayBearing)<10):
                        #drive straight
                        print("drive straight")
                        board.motor_movement([board.M1], board.CCW, duty_cycle)
                        board.motor_movement([board.M2], board.CW, duty_cycle)
                    elif(packingBayBearing>0):
                        print("shuffle right")
                        LED("red", "on")
                        board.motor_movement([board.M1], board.CCW, turning_duty_cycle)
                        #board.motor_movement([board.M2], board.CW, duty_cycle)
                    else:
                        print("shuffle left")
                        LED("green", "on")
                        #board.motor_movement([board.M1], board.CCW, duty_cycle)
                        board.motor_movement([board.M2], board.CW, turning_duty_cycle)                

            print("\n\n")

            k = cv2.waitKey(5) & 0xFF   # Make the program wait for 5ms before continuing (also required to display image).
            if k == 27: # Esc key
                vision.Dispose()
                led.Dispose
                cv2.destroyAllWindows()
                break
            

    # except KeyboardInterrupt as e:
    #     # attempt to stop motors from running
    finally:
        # Cleanup GPIO setup to reset pins
        board.motor_stop(board.ALL)   # stop all DC motor
        # servo.detach()
        # GPIO.cleanup()