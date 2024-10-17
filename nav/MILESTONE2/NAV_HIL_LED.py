#!/usr/bin/python

from led_test import LED
# import the packing bot module - this will include math, time, numpy (as np) and CoppeliaSim python modules
from warehousebot_lib import *
import time
#import any other required python modules
import csv


LED("red", "off")
LED("yellow", "off")
LED("green", "off")


with open("Order_1.csv", mode="r", encoding='utf-8-sig') as csv_file:
    csv_reader = csv.reader(csv_file)
    csv_list = list(csv_reader) #gets around strange csv reader object behaviour

# SET SCENE PARAMETERS
sceneParameters = SceneParameters()
for row in csv_list[1:]:
    sceneParameters.bayContents[int(row[1]), int(row[2]), int(row[3])] = warehouseObjects.rubiks
# Starting contents of the bays [shelf,X,Y]. Set to -1 to leave empty.
#sceneParameters.bayContents = np.random.random_integers(0,5,sceneParameters.bayContents.shape)

# sceneParameters.bayContents[0,3,1] = warehouseObjects.bowl
# sceneParameters.bayContents[1,1,2] = warehouseObjects.mug
# sceneParameters.bayContents[2,3,1] = warehouseObjects.bottle
# sceneParameters.bayContents[3,1,2] = warehouseObjects.soccer
# sceneParameters.bayContents[4,2,0] = warehouseObjects.rubiks
# sceneParameters.bayContents[5,0,1] = warehouseObjects.cereal


sceneParameters.obstacle0_StartingPosition = [0.2, -0.1]  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
# sceneParameters.obstacle0_StartingPosition = None  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
sceneParameters.obstacle1_StartingPosition = [0.8, -0.75]   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
sceneParameters.obstacle2_StartingPosition = [0.25, 0.75]   #

# sceneParameters.obstacle0_StartingPosition = -1 #[-0.5,0]  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
# # sceneParameters.obstacle0_StartingPosition = None  # starting position of obstacle 0 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
# sceneParameters.obstacle1_StartingPosition = -1   # starting position of obstacle 1 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene
# sceneParameters.obstacle2_StartingPosition = -1   # starting position of obstacle 2 [x, y] (in metres), -1 if want to use current CoppeliaSim position, or none if not wanted in the scene


# SET ROBOT PARAMETERS
robotParameters = RobotParameters()

# Drive Parameters
robotParameters.driveType = 'differential'	# specify if using differential or omni drive system
robotParameters.minimumLinearSpeed = 0.0  	# minimum speed at which your robot can move forward in m/s
robotParameters.maximumLinearSpeed = 0.3 	# maximum speed at which your robot can move forward in m/s
robotParameters.driveSystemQuality = 1		# specifies how good your drive system is from 0 to 1 (with 1 being able to drive in a perfectly straight line when told to do so)

# Camera Parameters
robotParameters.cameraOrientation = 'landscape' # specifies the orientation of the camera, either landscape or portrait
robotParameters.cameraDistanceFromRobotCenter = 0.1 # distance between the camera and the center of the robot in the direction of the collector in metres
robotParameters.cameraHeightFromFloor = 0.15 # height of the camera relative to the floor in metres
robotParameters.cameraTilt = 0.0 # tilt of the camera in radians

# Vision Processing Parameters
robotParameters.maxItemDetectionDistance = 1 # the maximum distance away that you can detect the items in metres
robotParameters.maxPackingBayDetectionDistance = 2.5 # the maximum distance away that you can detect the packing bay in metres
robotParameters.maxObstacleDetectionDistance = 1.5 # the maximum distance away that you can detect the obstacles in metres
robotParameters.maxRowMarkerDetectionDistance = 2.5 # the maximum distance away that you can detect the row markers in metres

# Collector Parameters
robotParameters.collectorQuality = 1 # specifies how good your item collector is from 0 to 1.0 (with 1.0 being awesome and 0 being non-existent)
robotParameters.maxCollectDistance = 0.15 #specificies the operating distance of the automatic collector function. Item needs to be less than this distance to the collector

robotParameters.sync = True # This parameter forces the simulation into syncronous mode when True; requiring you to call stepSim() to manually step the simulator in your loop

import time

#USE ME FOR THE ROBOT WHEN FULLY INTEGRATED
# def TicTocGenerator():
#     # Generator that returns time differences
#     ti = 0           # initial time
#     tf = time.time() # final time
#     while True:
#         ti = tf
#         tf = time.time()
#         yield tf-ti # returns the time difference

# TicToc = TicTocGenerator() # create an instance of the TicTocGen generator

# # This will be the main function through which we define both tic() and toc()
# def toc(tempBool=True):
#     # Prints the time difference yielded by generator instance TicToc
#     tempTimeInterval = next(TicToc)
#     if tempBool:
#         print( "Elapsed time: %f seconds.\n" %tempTimeInterval )

# def tic():
#     # Records a time in TicToc, marks the beginning of a time interval
#     toc(False)

def rotate(iterations):
    #print("starting")
    for i in range(abs(iterations)):
            warehouseBotSim.UpdateObjectPositions()
            warehouseBotSim.stepSim()
            warehouseBotSim.SetTargetVelocities(0, math.copysign(0.5, iterations))
    #print("finished rotating")
    warehouseBotSim.SetTargetVelocities(0, 0)

def collectItem(shelf, height): #height unused for now
    LED("yellow", "on")
    LED("red", "off")
    print("YELLOW LED: Collecting Item")
    if (shelf % 2) == 0:
        #print("The number is even (left turn)")
        rotate(87)
    else:
        #print("The number is odd (right turn)")
        rotate(-87)

    for i in range(10): 
        #print("YELLOW LED: Collecting Item")
        warehouseBotSim.SetTargetVelocities(0.2, 0)
        warehouseBotSim.UpdateObjectPositions()
        if robotParameters.sync:
            warehouseBotSim.stepSim()
    for i in range(10):
        #print("YELLOW LED: Collecting Item")
        warehouseBotSim.CollectItem(height)
        warehouseBotSim.SetTargetVelocities(0, 0)
        warehouseBotSim.UpdateObjectPositions()
        if robotParameters.sync:
            warehouseBotSim.stepSim()
    for i in range(10):
        #print("YELLOW LED: Collecting Item")
        warehouseBotSim.SetTargetVelocities(-0.2, 0)
        warehouseBotSim.UpdateObjectPositions()
        if robotParameters.sync:
            warehouseBotSim.stepSim()

    if (shelf % 2) == 0:
       # print ("The number is even (left turn)")
        rotate(88)
    else:
        #print ("The number is odd (right turn)")
        rotate(-88)
    #print("Finished Collecting Item")
    warehouseBotSim.SetTargetVelocities(0, 0)
    warehouseBotSim.UpdateObjectPositions()
    if robotParameters.sync:
        warehouseBotSim.stepSim()
    exitRow(1.25 - fetchItemProperty(focus_row)[1])


    
def exitRow(distance):
    LED("red", "on")
    LED("yellow", "off")
    print("RED LED: Searching for Item")
    for i in range(round((distance/(0.0002*50)))):
        # Get Detected Objects
        objectsRB = warehouseBotSim.GetDetectedObjects(
            [
                warehouseObjects.items,
                warehouseObjects.shelves,
                warehouseObjects.row_markers,
                warehouseObjects.obstacles,
                warehouseObjects.packingBay,
            ]
        )
        itemsRB, packingBayRB, obstaclesRB, rowMarkerRangeBearing, shelfRangeBearing = objectsRB

        dist = warehouseBotSim.readProximity()

        # Check to see if any shelves are within the camera's FOV
        shelfRange = 10
        shelfBearing = 0
        if shelfRangeBearing != None:
            # loop through each obstacle detected using Pythonian way
            for shelf in shelfRangeBearing:
                if shelf != None:
                    if shelf[0] < shelfRange:
                        shelfRange = shelf[0]
                        shelfBearing = shelf[1]
        #print("shelf (range, bearing): %0.4f, %0.4f"%(shelfRange, shelfBearing))
        warehouseBotSim.SetTargetVelocities(0.2, max(-shelfBearing, -0.5) if shelfBearing>0 else min(-shelfBearing, 0.5)) 
        warehouseBotSim.UpdateObjectPositions()
        if robotParameters.sync:
            warehouseBotSim.stepSim()

    global focus_row
    if(focus_row == 0):
        global row0ItemProperties
        #print("deleting"+str(row0ItemProperties[-1]) + "of" + str(focus_row))
        del row0ItemProperties[-1]
    elif(focus_row == 1):
        global row1ItemProperties
        #print("deleting"+str(row1ItemProperties[-1]) + "of" + str(focus_row))
        del row1ItemProperties[-1]
    elif(focus_row == 2):
        global row2ItemProperties
        #print("deleting"+str(row2ItemProperties[-1]) + "of" + str(focus_row))
        del row2ItemProperties[-1]

    #focus_row = choose_next_row()
    focus_row -= 1
    if(focus_row == -1):
        focus_row = 2

    global turn_right
    turn_right = True
    global state
    state = "lookingforpackingbay"
    global trigger_beginning_action
    trigger_beginning_action = True


def choose_next_row():

    # if(len(row0ItemProperties)>0):
    #     #return 0
    # # elif(len(row1ItemProperties)>0):
    # #     #return 1
    # # elif(len(row2ItemProperties)>0):
    # #     #return 2
    # else:
    #     state = "finished"
    global focus_row
    focus_row -= 1
    if(focus_row == -1):
        focus_row = 2
        

#returns shelf number, bay number, and height of an item in the selected row
def fetchItemProperty(row):
    # print("looking in row " + str(row))
    # print(row0ItemProperties)
    # print(row1ItemProperties)
    # print(row2ItemProperties)
    if(row == 0):
        return_item = row0ItemProperties[-1]
        return(return_item)
    elif(row == 1):
        return_item = row1ItemProperties[-1]
        return(return_item)
    elif(row == 2):
        return_item = row2ItemProperties[-1]
        return(return_item)
    
focus_row = 1

turn_right = False
checkpoint1 = False
row_distance_to_packingbay = [0.24, 0.85, 1.46]
bay_depth = [0.87, 0.58, 0.33, 0.08]
estimated_distance_to_packingbay = 0

old_packingBayRB = None
state = "lookingforpackingbay"

dt = 0.05 #assume 50 ms dt for simulator
trigger_beginning_action = True

# MAIN SCRIPT
if __name__ == '__main__':

    # Wrap everything in a try except case that catches KeyboardInterrupts. 
    # In the exception catch code attempt to Stop the CoppeliaSim so don't have to Stop it manually when pressing CTRL+C
    try:
        #converts shelf number into row number and side
        import array
        row0ItemProperties = []
        row1ItemProperties = []
        row2ItemProperties = []
        for item in csv_list[1:]:
            #print("row" + str(int(item[1])>>1) + " side " + str(int(item[1]) % 2))

            new_array = [int(item[1]), bay_depth[int(item[2])], int(item[3])]
            if(int(item[1])>>1 == 0):
                row0ItemProperties.append(new_array)

            if(int(item[1])>>1 == 1):
                row1ItemProperties.append(new_array)

            if(int(item[1])>>1 == 2):
                row2ItemProperties.append(new_array)

        print(row0ItemProperties)
        print(row1ItemProperties)
        print(row2ItemProperties)


        # Create CoppeliaSim PackerBot object - this will attempt to open a connection to CoppeliaSim. Make sure CoppeliaSim is running.
        warehouseBotSim = COPPELIA_WarehouseRobot('10.88.31.252', robotParameters, sceneParameters) #10.88.31.252 #10.88.46.34
        warehouseBotSim.StartSimulator()

        LED("red", "on")
        LED("yellow", "off")
        print("RED LED: Searching for Item")
        while True:
            # move the robot at a forward velocity of 0.05m/s with a rotational velocity of 0.1 rad/s.
            

            warehouseBotSim.UpdateObjectPositions()

            # Get Detected Objects
            objectsRB = warehouseBotSim.GetDetectedObjects(
                [
                    warehouseObjects.items,
                    warehouseObjects.shelves,
                    warehouseObjects.row_markers,
                    warehouseObjects.obstacles,
                    warehouseObjects.packingBay,
                ]
            )
            itemsRB, packingBayRB, obstaclesRB, rowMarkerRangeBearing, shelfRangeBearing = objectsRB

            dist = warehouseBotSim.readProximity()

            if(packingBayRB != None):
                old_packingBayRB = packingBayRB

            #Check to see if an item is within the camera's FOV
            for itemClass in itemsRB:
                if itemClass != None:
                    # loop through each item detected using Pythonian way
                    for itemRB in itemClass:
                        itemRange = itemRB[0]
                        itemBearing = itemRB[1]

            rowNumber=0
            for rowMarker in rowMarkerRangeBearing:
                rowNumber+=1
                #print(rowMarker, i)
                if rowMarker != None:
                    rowMarkerRange = rowMarker[0]
                    rowMarkerBearing = rowMarker[1]

            # Check to see if any shelves are within the camera's FOV
            shelfRange = 10
            shelfBearing = 0
            if shelfRangeBearing != None:
                # loop through each obstacle detected using Pythonian way
                for shelf in shelfRangeBearing:
                    if shelf != None:
                        if shelf[0] < shelfRange:
                            shelfRange = shelf[0]
                            shelfBearing = shelf[1]
            #print("shelf (range, bearing): %0.4f, %0.4f"%(shelfRange, shelfBearing))

            # Check to see if any obstacles are within the camera's FOV
            if obstaclesRB != None:
                # loop through each obstacle detected using Pythonian way
                for obstacle in obstaclesRB:
                    obstacleRange = obstacle[0]
                    obstacleBearing = obstacle[1]
                    #print("OBSTACLE (range, bearing): %0.4f, %0.4f"%(obstacle[0], obstacle[1]))

            # BEGINNING OF State Machine
            if(state == "lookingforpackingbay"):
                if (packingBayRB == None):
                    warehouseBotSim.SetTargetVelocities(0, 1)
                else:
                    if(trigger_beginning_action):
                        if(packingBayRB[0] < row_distance_to_packingbay[focus_row]):
                            backwards_distance_required = (row_distance_to_packingbay[focus_row] - packingBayRB[0]) * 1.4
                            backwards_loops_required = round((backwards_distance_required/(0.0002*50)))
                            rotate(75)
                            print("turningawayfrompackingbay")
                            turn_right = False
                            state = "turningawayfrompackingbay"
                            continue
                        else:
                            trigger_beginning_action = False
                    else:
                        warehouseBotSim.SetTargetVelocities(0.3, 0)

                        if(old_packingBayRB[0] < row_distance_to_packingbay[focus_row]):
                            old_packingBayRB = None
                            print("real")
                            state = "drivingdownrow"
                if(turn_right):
                    warehouseBotSim.SetTargetVelocities(0, -1)
                    if(packingBayRB != None):
                        checkpoint1 = True
                    if(packingBayRB == None and checkpoint1 == True):
                        old_packingBayRB = None
                        turn_right = False
                        checkpoint1 = False
                else:
                    if(old_packingBayRB != None):
                        if(old_packingBayRB[0] < 0.8):
                            if(estimated_distance_to_packingbay < row_distance_to_packingbay[focus_row]):
                                old_packingBayRB = None
                                print("estimated")
                                state = "drivingdownrow"
                            warehouseBotSim.SetTargetVelocities(0.3, 0)
                            #print("switching to estimate" + str(estimated_distance_to_packingbay))
                            distance_travelled_this_tick = 0.3*dt
                            estimated_distance_to_packingbay -= distance_travelled_this_tick
                            #print(estimated_distance_to_packingbay)
                        else:
                            distance_travelled_this_tick = 0
                            estimated_distance_to_packingbay = old_packingBayRB[0]

            else:
                trigger_beginning_action = True
                if(state == "drivingdownrow"):
                    if not all(v is None for v in rowMarkerRangeBearing):
                        if(rowMarkerBearing > -0.1):
                            warehouseBotSim.SetTargetVelocities(0.2, rowMarkerBearing)
                        else:
                            warehouseBotSim.SetTargetVelocities(0, -1)
                        if rowMarkerRange < fetchItemProperty(focus_row)[1]:
                            collectItem(fetchItemProperty(focus_row)[0],fetchItemProperty(focus_row)[2])
                    else:
                        warehouseBotSim.SetTargetVelocities(0, -1)
                elif(state == "turningawayfrompackingbay"):
                    if(abs(shelfBearing)==0):
                        warehouseBotSim.SetTargetVelocities(0, 1)
                    else:
                        i = 0
                        state = "drivingawayfrompackingbay"
                elif(state == "drivingawayfrompackingbay"):
                    i+=1
                    if(abs(shelfBearing)==0):
                        warehouseBotSim.SetTargetVelocities(0.2, 0.5)
                    else:
                        warehouseBotSim.SetTargetVelocities(0.2, -0.5)
                    if(i>backwards_loops_required):
                        old_packingBayRB = None
                        state = "lookingforpackingbay"

            # Update object Positions
            if robotParameters.sync:
                warehouseBotSim.stepSim()
            
    except KeyboardInterrupt as e:
        # attempt to stop simulator so it restarts and don't have to manually press the Stop button in CoppeliaSim 
        warehouseBotSim.StopSimulator()