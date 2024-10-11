# -*- coding:utf-8 -*-
'''!
  @file DC_Motor_Demo.py
  @brief Connect board with raspberryPi.
  @n Make board power and motor connection correct.
  @n Run this demo.
  @n Motor 1 will move slow to fast, orientation clockwise, 
  @n motor 2 will move fast to slow, orientation count-clockwise, 
  @n then fast to stop. loop in few seconds.
  @n Motor speed will print on terminal
  @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license    The MIT License (MIT)
  @author     [tangjie](jie.tang@dfrobot.com)
  @version    V1.0.1
  @date       2022-04-19
  @url  https://github.com/DFRobot/DFRobot_RaspberryPi_Motor
'''
from __future__ import print_function
import sys
import os
sys.path.append("../")

import time

from DFROBOT_MOTOR_CODE import THIS_BOARD_TYPE, DFRobot_DC_Motor_IIC as Board

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

  duty_cycle = 35
  # cd DFRobot_RaspberryPi_Motor/examples/
  # python DC_Motor_Demo.py


  # while True:
  #   board.motor_movement([board.M1], board.CW, duty_cycle)    # Motor 1 clockwise
  #   board.motor_movement([board.M2], board.CCW, duty_cycle)   # Motor 2 counterclockwise
  #   time.sleep(1)
  #   speed = board.get_encoder_speed(board.ALL)
  #   print("Duty cycle: %d%%, M1 encoder speed: %d rpm, M2 encoder speed %d rpm" % (duty_cycle, speed[0], speed[1]))
  #   time.sleep(2)
  #   # for duty in range(5, 95, 1):   # slow to fast
  #   #   board.motor_movement([board.M1], board.CW, 50)    # DC motor 1 movement, orientation clockwise
  #   #   board.motor_movement([board.M2], board.CCW, 50)   # DC motor 2 movement, orientation count-clockwise
  #   #   time.sleep(0.1)
  #   #   speed = board.get_encoder_speed(board.ALL)      # Use boadrd.all to get all encoders speed
  #   #   print("duty: %d, M1 encoder speed: %d rpm, M2 encoder speed %d rpm" %(50, speed[0], speed[1]))

  #   # for duty in range(95, 5, - 10):   # fast to low
  #   #   board.motor_movement([board.M1], board.CW, duty)    # DC motor 1 movement, orientation clockwise
  #   #   board.motor_movement([board.M2], board.CCW, duty)   # DC motor 2 movement, orientation count-clockwise
  #   #   time.sleep(1)
  #   #   speed = board.get_encoder_speed(board.ALL)      # Use boadrd.all to get all encoders speed
  #   #   print("duty: %d, M1 encoder speed: %d rpm, M2 encoder speed %d rpm" %(duty, speed[0], speed[1])
  #   print("stop all motor")
  #   board.motor_stop(board.ALL)   # stop all DC motor
  #   print_board_status()
  #   time.sleep(1)
