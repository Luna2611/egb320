import numpy as np
import cv2
from vision import Vision

    
if __name__ == "__main__":   
    vision = Vision()
    vision.SetupCamera()

    while(1):
        itemsBearing, obstaclesRangeBearing, packingBayBearing, bayMarkerRangeBearing, rowMarkersRangeBearing, shelfBearing, wallRange = vision.Run()

        print("\n\n")

        if wallRange is not None:
            print("Range from wall: " + str(wallRange))

        if len(itemsBearing) > 0:
            print( "Amount of items: " + str(len(itemsBearing)) )
            for item in itemsBearing:
                print("Item Bearing: " + str(item))
        
        if len(obstaclesRangeBearing) > 0:
            print( "Amount of obstacles:" + str(len(obstaclesRangeBearing)) )
            for obstacle in obstaclesRangeBearing:
                print("Obstacle Distance: " + str(obstacle[0]))
                print("Obstacle Bearing: " + str(obstacle[1]))
 
        if packingBayBearing is not None:
            print("Packing Bay Bearing: " + str(packingBayBearing))

        if len(bayMarkerRangeBearing) > 0:
            print("Packing Bay Marker Range: " + str(bayMarkerRangeBearing[0]))
            print("Packing Bay Marker Bearing: " + str(bayMarkerRangeBearing[1]))

        if len(rowMarkersRangeBearing) > 0:
            for rowMarker in rowMarkersRangeBearing:
                if rowMarker:
                    print("This is row: " + str(rowMarker[0]))
                    print("Row Marker Range: " + str(rowMarker[1]))
                    print("Row Marker Bearing: " + str(rowMarker[2]))

        if len(shelfBearing) > 0:
            print( "Amount of shelves: " + str(len(shelfBearing)) )
            i = 0
            for shelf in shelfBearing:            
                print("Shelf Bearing: " + str(shelfBearing[i]))
                i+=1

        print("\n\n")

        k = cv2.waitKey(5) & 0xFF   # Make the program wait for 5ms before continuing (also required to display image).
        if k == 27: # Esc key
            vision.Dispose()
            cv2.destroyAllWindows()
            break

        