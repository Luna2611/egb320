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

#LED stuff
import RPi.GPIO as GPIO			# Import the GPIO module
import time 				# Import the time module
GPIO.setmode(GPIO.BCM)			# Set the GPIO pin naming convention to BCM
GPIO.setup(21,GPIO.OUT)			# Set up GPIO pin 21 as an output
GPIO.output(21,GPIO.HIGH) 		# Set GPIO pin 21 to digital high (on)
time.sleep(5)				# Wait for 5 seconds
GPIO.output(21,GPIO.LOW)		# Set GPIO pin 21 to digital low (off)
GPIO.cleanup()				# Exit the GPIO session cleanly


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

 
# switch out for any other parameters 
duty = 10
duration = 2

#hayden's attempt at stuff: ignore mostly
# def piSetTargetVelocity(forward_velocity, rotation_velocity):
#   if(rotational_velocity > 0):
#     board.motor_movement([board.M1], board.CCW, rotation_velocity - forward_velocity)    # Motor 1 backwards (CCW)
#     board.motor_movement([board.M2], board.CW, rotation_velocity + forward_velocity)    # Motor 2 forward (CW)




# def move_foward(duty, duration):
#   board.motor_movement([board.M1], board.CW, duty)    # Motor 1 forward (CW)
#   board.motor_movement([board.M2], board.CW, duty)    # Motor 2 forward (CW)
#   time.sleep(1)
#   board.motor_stop(board.ALL)

# def move_backward(duty, duration):
#   board.motor_movement([board.M1], board.CCW, duty)    # Motor 1 backwards (CCW)
#   board.motor_movement([board.M2], board.CCW, duty)    # Motor 2 backwards (CCW)
#   time.sleep(1)
#   board.motor_stop(board.ALL)

# def spin_left(duty, duration):
#   board.motor_movement([board.M1], board.CCW, duty)    # Motor 1 backwards (CCW)
#   board.motor_movement([board.M2], board.CW, duty)    # Motor 2 forward (CW)
#   time.sleep(1)
#   board.motor_stop(board.ALL)

# def spin_right(duty, duration):
#   board.motor_movement([board.M1], board.CW, duty)    # Motor 1 forward (CW)
#   board.motor_movement([board.M2], board.CCW, duty)    # Motor 2 backwards (CW)
#   time.sleep(1)
#   board.motor_stop(board.ALL)



# General drive function for controlling speed and rotation speed
def drive(speed, rotation_speed):
    wheel_radius = 0.035
    wheel_base = 0.151
    
    left_wheel_speed = (speed - 0.5 * rotation_speed * wheel_base) / wheel_radius
    right_wheel_speed = (speed + 0.5 * rotation_speed * wheel_base) / wheel_radius
    
    max_speed = 0.3 / wheel_radius
    min_speed = 0.03 / wheel_radius

    # if speed == 0 and rotation_speed != 0:
    #     left_wheel_speed = -(rotation_speed * wheel_base / 2) / wheel_radius
    #     right_wheel_speed = (rotation_speed * wheel_base / 2) / wheel_radius
    
    # Ensure the minimum speed threshold
    if speed != 0:
      if abs(left_wheel_speed) < min_speed:
          left_wheel_speed = 0
      if abs(right_wheel_speed) < min_speed:
          right_wheel_speed = 0

    duty_left = int((left_wheel_speed / max_speed) * 100)
    duty_right = int((right_wheel_speed / max_speed) * 100)

    # Clamp duty cycles to [-100, 100]
    duty_left = max(min(duty_left, 100), -100)
    duty_right = max(min(duty_right, 100), -100)

    # if speed == 0.03:
    #   duty_left = 25
    #   duty_right = 25

    # Print for debugging
    print(f"Speed: {speed}, Rotation Speed: {rotation_speed}")
    print(f"Left Wheel Speed: {left_wheel_speed}, Right Wheel Speed: {right_wheel_speed}")
    print(f"Duty Left: {duty_left}, Duty Right: {duty_right}")
    
    if duty_left >= 0:
        board.motor_movement([board.M1], board.CCW, duty_left)
    else:
        board.motor_movement([board.M1], board.CW, abs(duty_left))

    if duty_right >= 0:
        board.motor_movement([board.M2], board.CW, duty_right)
    else:
        board.motor_movement([board.M2], board.CCW, abs(duty_right))

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
    print("board begin faild")
    time.sleep(2)
  print("board begin success")

  board.set_encoder_enable(board.ALL)                 # Set selected DC motor encoder enable
  # board.set_encoder_disable(board.ALL)              # Set selected DC motor encoder disable
  board.set_encoder_reduction_ratio(board.ALL, 43)    # Set selected DC motor encoder reduction ratio, test motor reduction ratio is 43.8

  board.set_moter_pwm_frequency(1000)   # Set DC motor pwm frequency to 1000HZ

  # cd DFRobot_RaspberryPi_Motor-master_john/DFRobot_RaspberryPi_Motor-master/examples/
  # python DC_Motor_Demo_v2.py

  # high speed 0.18, 0 , do 2 - 3 seconds
  # low speed 0.03, 0 , do 10-12 seconds
  # rotate 90 left 0, 3.6 , do 2 seconds
  # rotate 90 right 0, -3.6 do 2 seconds
  # 15 incline 0.08 works (not 1kg)
  # loop to test if motors spin the right way


  ########
  # try:
  #   while True: 
  #       drive(0, -3.8)
  #       speed = board.get_encoder_speed(board.ALL)      # Use boadrd.all to get all encoders speed
  #       print("duty: %d, M1 encoder speed: %d rpm, M2 encoder speed %d rpm" %(duty, speed[0], speed[1]))
  #       # print("Duty cycle: %d%%, M1 encoder speed: %d rpm, M2 encoder speed %d rpm" % (duty, speed[0], speed[1]))
  #       print("stop all motor")
  #       time.sleep(2)

  #       # speed = board.get_encoder_speed(board.ALL)      # Use boadrd.all to get all encoders speed
  #       # print("duty: %d, M1 encoder speed: %d rpm, M2 encoder speed %d rpm" %(duty, speed[0], speed[1]))
  #       # print("stop all motor")
  #       board.motor_stop(board.ALL)   # stop all DC motor
  #       print_board_status()
  #       time.sleep(2)
  # except KeyboardInterrupt as e:
  # # attempt to stop motors from running
  #   board.motor_stop(board.ALL)   # stop all DC motor