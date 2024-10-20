from __future__ import print_function

import sys
import csv

with open("Order_1.csv", mode="r", encoding='utf-8-sig') as csv_file:
    csv_reader = csv.reader(csv_file)
    csv_array = list(csv_reader) #gets around strange csv reader object behaviour

# csv_array = []
# for row in csv_string_array:
#     int_row = []
#     for element in row:
#         int_row.append(int(element))  # Convert string to integer
#     csv_array.append(int_row)

# print(csv_array)

item_index = 1

print(csv_array[item_index][2])#[item_index])

#sys.exit("Stopping the script here")

import numpy as np
import cv2
from vision import Vision
# from item_collection import grab
# from item_collection import shelfHeight
# from led_test import LED


# Item management
from Grabber import Grabber  # Import the grab module

from Lifter import Lifter 

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
duty_cycle = 65
turning_duty_cycle = 90
slow_turn_speed = 70
state = "turnrightuntilshelf" # liningupwithrow
if __name__ == "__main__":   
    grabber = Grabber()
    lifter = Lifter()
    vision = Vision()
    vision.SetupCamera()
    lifter.set_height(0)
    try:
        while(1):
            #lifter.set_height(0)
            itemsBearing, obstaclesRangeBearing, packingBayBearing, bayMarkerRangeBearing, rowMarkersRangeBearing, shelfBearing, wallRange = vision.Run()

            print("\n\n")
            print(wallRange)
            # if len(itemsBearing) > 0:
            #     print( "Amount of items: " + str(len(itemsBearing)))
            #     for item in itemsBearing:
            #         print("Item Bearing: " + str(item))
            
            # if len(obstaclesRangeBearing) > 0:
            #     print( "Amount of obstacles:" + str(len(obstaclesRangeBearing)) )
            #     for obstacle in obstaclesRangeBearing:
            #         print("Obstacle Bearing: " + str(obstacle[0]))
            #         print("Obstacle Distance: " + str(obstacle[1]))

            
            #     print("Packing Bay Bearing: " + str(packingBayBearing))

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
                    print("This is row: " + str(rowMarker[0]))
                    print("Row Marker Range: " + str(rowMarker[1]))
                    print("Row Marker Bearing: " + str(rowMarker[2]))
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




#-----------------------------------------------------------------------------------------------------------------------
            #### BEGIN STATE MACHINE
            print(state)
            if(state == "parked"):
                # grabber.Grab(0)
                # time.sleep(1)
                # grabber.Grab(1)
                # time.sleep(2)
                time.sleep(0.1)
            elif(state == "turnrightuntilshelf"):
                if(shelfBearing):
                    state = "turnleftuntilNOshelf"
                else:
                    board.motor_movement([board.M1], board.CCW, turning_duty_cycle)
                    board.motor_movement([board.M2], board.CCW, turning_duty_cycle)
            elif(state == "turnleftuntilNOshelf"):
                if(shelfBearing):
                    board.motor_movement([board.M1], board.CW, turning_duty_cycle)
                    board.motor_movement([board.M2], boardCW, turning_duty_cycle)
                else:
                    state = "drivetorow1"
            elif(state == "drivetorow1"):
                if(shelfBearing):
                    board.motor_movement([board.M1], board.CW, duty_cycle/2)
                    board.motor_movement([board.M2], board.CW, duty_cycle)
                else:
                    board.motor_movement([board.M1], board.CCW, duty_cycle)
                    board.motor_movement([board.M2], board.CCW, duty_cycle/2)
            elif(state == "drivingoutrow"):
                #lifter.set_height(0)
                if(seen_rowmarker and seen_rowmarker[1] > 700):
                    state = "parked"
                if(seen_rowmarker and seen_rowmarker[1] < 500):
                    targetBearing = seen_rowmarker[2]
                    # if(1):
                elif(len(shelfBearing) == 2):
                    targetBearing = (shelfBearing[0] + shelfBearing[1]) / 2
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
                board.motor_movement([board.M1], board.CW, duty_cycle)
                board.motor_movement([board.M2], board.CCW, duty_cycle)
                time.sleep(0.5)
                board.motor_movement([board.M1], board.CCW, 100)
                board.motor_movement([board.M2], board.CCW, 100)
                time.sleep(0.6)
                board.motor_movement([board.M1], board.CW, duty_cycle)
                board.motor_movement([board.M2], board.CCW, duty_cycle)
                time.sleep(1)
                # board.motor_stop(board.ALL)   # stop all DC motor
                
                lifter.set_height(0)
                state = "drivingoutrow"

            elif(state == "openloopturningtoshelf"):
                # print("GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG")
                # grab(1)
                # time.sleep(10)
                # grab(2)

                lifter.set_height(2)
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
                # lifter.set_height(1)
                bay_lookup = [1550, 1000, 750]
                print(bay_lookup[int(csv_array[item_index][2])])
                #print(len(shelfBearing))
                if(int(csv_array[item_index][2]) != 3):
                    if(seen_rowmarker and seen_rowmarker[1] < bay_lookup[int(csv_array[item_index][2])]):

                        # if(seen_rowmarker[1] < 1600): # second bay is 450 first bay is 800 0'th bay is 1150

                        state = "openloopturningtoshelf"
                else:
                    if(wallRange == "near"):
                        state = "openloopturningtoshelf"
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
                        board.motor_movement([board.M1], board.CCW, turning_duty_cycle)
                        board.motor_movement([board.M2], board.CCW, duty_cycle/2)
                    else:
                        print("shuffle left")
                        board.motor_movement([board.M1], board.CW, duty_cycle/2)
                        board.motor_movement([board.M2], board.CW, turning_duty_cycle)
            elif(state == "liningupwithrow"):
                #turn on spot left
                
                print("turn on spot left")
                board.motor_movement([board.M1], board.CW, slow_turn_speed)
                board.motor_movement([board.M2], board.CW, slow_turn_speed)                
                if(packingBayBearing):
                    if(abs(packingBayBearing)<10):
                        if(bayMarkerRangeBearing and bayMarkerRangeBearing[0] < 1450): #
                            print("PARKEDPARKEDPARKEDPARKEDPARKEDPARKEDPARKEDPARKED")
                            state = "parked"
                            board.motor_stop(board.ALL)   # stop all DC motor
                        #drive straight
                        print("drive straight")
                        board.motor_movement([board.M1], board.CCW, slow_turn_speed)
                        board.motor_movement([board.M2], board.CW, slow_turn_speed)
                    elif(packingBayBearing>0):
                        print("shuffle right")
                        # LED("red", "on")
                        board.motor_movement([board.M1], board.CCW, slow_turn_speed)
                        board.motor_movement([board.M2], board.CW, slow_turn_speed/1.5)
                    else:
                        print("shuffle left")
                        # LED("green", "on")
                        board.motor_movement([board.M1], board.CCW, slow_turn_speed/1.5)
                        board.motor_movement([board.M2], board.CW, slow_turn_speed) 

                elif(state == 'localising'):
                    if packingBayBearing or bayMarkerRangeBearing:
                        board.motor_movement([board.M1], board.CCW, slow_turn_speed)
                        board.motor_movement([board.M2], board.CW, slow_turn_speed)
                    elif rowMarkersRangeBearing:
                       state = "drivingdownrow "        

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