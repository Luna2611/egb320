from __future__ import print_function

import numpy as np
import cv2
import sys
from vision import Vision
sys.path.append('/home/group36/egb320/')
#/home/group36/egb320/vision
print(str(sys.path))

from led import init
init()
from led import LED
from ..mobility import DRIVE_FUNCTION

### Motor INitialisation

import sys
import os
sys.path.append("../")

import time

from mobility.DFROBOT_MOTOR_CODE import THIS_BOARD_TYPE, DFRobot_DC_Motor_IIC as Board

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
state = "lookingforpackingbay"
if __name__ == "__main__":   
    vision = Vision()
    vision.SetupCamera()
    try:
        while(1):
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

            #if len(rowMarkersRangeBearing) > 0:
            #print("cant see shit")
            seen_rowmarker = None
            board.motor_stop(board.ALL)   # stop all DC motor
            LED("all", "off")
            for rowMarker in rowMarkersRangeBearing:
                if rowMarker:
                    # print("This is row: " + str(rowMarker[0]))
                    # print("Row Marker Range: " + str(rowMarker[1]))
                    # print("Row Marker Bearing: " + str(rowMarker[2]))
                    seen_rowmarker = rowMarker
                    LED("yellow", "on")

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
            if(state == "lookingforpackingbay"):
                # #turn on spot left
                # print("turn on spot left")
                # board.motor_movement([board.M1], board.CW, turning_duty_cycle)
                # board.motor_movement([board.M2], board.CW, turning_duty_cycle)                
                # if(packingBayBearing):
                #     board.motor_stop(board.ALL)   # stop all DC motor
                #     if(abs(packingBayBearing)<10):
                #         #drive straight
                #         print("drive straight")
                #         board.motor_movement([board.M1], board.CCW, duty_cycle)
                #         board.motor_movement([board.M2], board.CW, duty_cycle)
                #     elif(packingBayBearing>0):
                #         print("shuffle right")
                #         LED("red", "on")
                #         board.motor_movement([board.M1], board.CCW, turning_duty_cycle)
                #         #board.motor_movement([board.M2], board.CW, duty_cycle)
                #     else:
                #         print("shuffle left")
                #         LED("green", "on")
                #         #board.motor_movement([board.M1], board.CCW, duty_cycle)
                #         board.motor_movement([board.M2], board.CW, turning_duty_cycle)

                    if(bayMarkerRangeBearing and bayMarkerRangeBearing[0] < 500):
                        LED("all", "on")
                        #state == "drivingdownrow"


                # #turn on spot left
                # print("turn on spot left")
                # board.motor_movement([board.M1], board.CW, turning_duty_cycle)
                # board.motor_movement([board.M2], board.CW, turning_duty_cycle)
                # if bayMarkerRangeBearing:
                #     if(bayMarkerRangeBearing[1] > -50):
                #       #drive straight
                #       print("drive right")
                #       board.motor_movement([board.M1], board.CCW, turning_duty_cycle)
                #       ####board.motor_movement([board.M2], board.CCW, turning_duty_cycle)
                #     else:
                #       print("drive straight")
                #       board.motor_movement([board.M1], board.CCW, duty_cycle)
                #       board.motor_movement([board.M2], board.CW, duty_cycle)


            if(state == "drivingdownrow"):
              if(seen_rowmarker):
                  if(abs(seen_rowmarker[2])<10):
                      #drive straight
                      print("drive straight")
                      board.motor_movement([board.M1], board.CCW, duty_cycle)
                      board.motor_movement([board.M2], board.CW, duty_cycle)
                  elif(seen_rowmarker[2]>0):
                      print("shuffle right")
                      board.motor_movement([board.M1], board.CCW, duty_cycle)
                      board.motor_movement([board.M2], board.CCW, duty_cycle/2)
                  else:
                      print("shuffle left")
                      board.motor_movement([board.M1], board.CW, duty_cycle/2)
                      board.motor_movement([board.M2], board.CW, duty_cycle)
              else:
                  #turn on spot right
                  print("turn on spot right")
                  board.motor_movement([board.M1], board.CCW, turning_duty_cycle)
                  board.motor_movement([board.M2], board.CCW, turning_duty_cycle)

            print("\n\n")

            k = cv2.waitKey(5) & 0xFF   # Make the program wait for 5ms before continuing (also required to display image).
            if k == 27: # Esc key
                vision.Dispose()
                cv2.destroyAllWindows()
                break
            

    # except KeyboardInterrupt as e:
    #     # attempt to stop motors from running
    finally:
        # Cleanup GPIO setup to reset pins
        board.motor_stop(board.ALL)   # stop all DC motor
        # GPIO.cleanup()