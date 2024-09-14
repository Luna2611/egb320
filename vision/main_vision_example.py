import numpy as np
import cv2
from Vision import Vision

    
if __name__ == "__main__":   
    vision = Vision()
    vision.SetupCamera()

    while(1):
        itemsBearing, obstaclesRangeBearing, packingBayBearing, bayMarkerRangeBearing, rowMarkersRangeBearing, shelfBearing = vision.Run()

        print("\n\n")

        print( "Amount of items: " + str(len(itemsBearing)) )
        for item in itemsBearing:
            print("Item Bearing: " + str(item))
        
        print( "Amount of obstacles:" + str(len(obstaclesRangeBearing)) )
        for obstacle in obstaclesRangeBearing:
            print("Obstacle Bearing: " + str(obstacle[0]))
            print("Obstacle Distance: " + str(obstacle[1]))

        print("Packing Bay Bearing: " + str(packingBayBearing))

        if len(bayMarkerRangeBearing) > 0:
            print("Packing Bay Marker Range: " + str(bayMarkerRangeBearing[0]))
            print("Packing Bay Marker Bearing: " + str(bayMarkerRangeBearing[1]))

        for rowMarker in rowMarkersRangeBearing:
            if rowMarker:
                print("This is row: " + str(rowMarker[0]))
                print("Row Marker Range: " + str(rowMarker[1]))
                print("Row Marker Bearing: " + str(rowMarker[2]))

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

        