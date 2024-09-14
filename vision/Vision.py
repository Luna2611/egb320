import cv2
import picamera2
import numpy as np
import time

from Distance import Distance
from Bearing import Bearing

from MarkerContour import MarkerContour
from ItemContour import ItemContour
from ObstacleContour import ObstacleContour
from ShelfContour import ShelfContour
from BayContour import BayContour

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
#320, 240
#640, 480

FOCAL_LENGTH = 656.2936301366 #px
#705.26
#1381.293630136656

HORIZONTAL_FOV = 62.2 #deg

class Vision:
    # HSV Thresholds (wall)    
    #lower_white = np.array([89, 0, 0])
    #upper_white = np.array([131, 68, 254])
    
    # Row marker measurements
    marker_radius = 70 #mm

    # Rubic measurements
    #known_width = 38 #mm
    #known_height = 38 #mm

    # Obstacle measurements
    obstacle_width = 50 #mm
    obstacle_height = 150 #mm
    

    cap = None

    def __init__(self):
        self.object_b = Bearing(FRAME_WIDTH, HORIZONTAL_FOV, FOCAL_LENGTH)

    def SetupCamera(self):
        # Create a camera object
        self.cap = picamera2.Picamera2()
        
        config = self.cap.create_video_configuration(main={"format":'RGB888',"size":(FRAME_WIDTH, FRAME_HEIGHT)})
        self.cap.configure(config)

        #start the camera
        self.cap.start()
    
    # Process frame from video feed, keep this in a while loop
    def Run(self): 

        itemBearing = []
        packingBayBearing = 0
        obstaclesRangeBearing = []
        rowMarkerRangeBearing = [None,None,None]
        shelfBearing = []
        bayMarkerRangeBearing = []

        t1 = time.time()                     # for measuring fps
      
        flipped_rgb_frame = self.cap.capture_array()          # capture a single image frame
        hsv_frame, rgb_frame = self.__frameConfig(flipped_rgb_frame)
        img = rgb_frame

#------------------------------------------------------------------------------------------------
        # Create a marker detection object
        marker_c = MarkerContour()
        marker_contours, bay_marker, row_markers = marker_c.GetContour(hsv_frame)

        marker_d = Distance(self.marker_radius, self.marker_radius , FOCAL_LENGTH)        
            
        if len(row_markers) > 0:
            # Contour 
            cv2.drawContours(img, marker_contours, -1, (0, 255, 0), 3)   

            # Bearing
            row_marker_bearing = round(self.object_b.GetBearing(row_markers[2]), 1)
            img = cv2.putText(img, str(row_marker_bearing), 
                                (row_markers[2] + 10, row_markers[3]), 
                                cv2.FONT_HERSHEY_SIMPLEX,  
                                0.5, (0, 0, 0))
            img = cv2.circle(img, (row_markers[2], row_markers[3]), 5, (0, 255, 0), 5)
            # Distance
            row_marker_d = round(marker_d.GetDistance(row_markers[1], row_markers[1]), 2)
            img = cv2.putText(img, str(row_marker_d), 
                                  (row_markers[2], row_markers[3]-20), 
                                    cv2.FONT_HERSHEY_SIMPLEX,  
                                    0.5, (0, 255, 0))
            img = cv2.putText(img, str(row_markers[0]+1), 
                                  (row_markers[2], row_markers[3]+20), 
                                    cv2.FONT_HERSHEY_SIMPLEX,  
                                    0.5, (0, 0, 255))
            if row_markers[0] < 3:
                index = row_markers[0]
                rowMarkerRangeBearing[index] = [row_markers[0]+1, row_marker_d, row_marker_bearing] 

        elif len(bay_marker) > 0:
            bay_marker_corner = bay_marker[0]
            bay_marker_dim = bay_marker[1]
            bay_marker_centroid = bay_marker[2]

            # Contour
            img = cv2.rectangle(img, (bay_marker_corner[0], bay_marker_corner[1]), 
                                (bay_marker_corner[0]+bay_marker_dim[0], bay_marker_corner[1]+bay_marker_dim[1]),
                                (0, 255, 0), 2)            
                     
            # Bearing
            bay_marker_bearing = round(self.object_b.GetBearing(bay_marker_centroid[0]), 1)
            img = cv2.putText(img, str(bay_marker_bearing), 
                                (bay_marker_centroid[0], bay_marker_centroid[1]), 
                                cv2.FONT_HERSHEY_SIMPLEX,  
                                0.5, (0, 0, 0))
            img = cv2.circle(img, (bay_marker_centroid[0], bay_marker_centroid[1]), 5, (0, 255, 0), 5)

            # Distance
            bay_marker_d = round(marker_d.GetDistance(bay_marker_dim[0], bay_marker_dim[1]), 2)
            img = cv2.putText(img, str(bay_marker_d), 
                                  (bay_marker_centroid[0], bay_marker_centroid[1]-20), 
                                    cv2.FONT_HERSHEY_SIMPLEX,  
                                    0.5, (0, 255, 0))
            bayMarkerRangeBearing =  [bay_marker_d, bay_marker_bearing]

#------------------------------------------------------------------------------------------------
        # Create an bay detection object
        bay_c = BayContour()
        bay_contour, bay = bay_c.GetContour(hsv_frame)

        if len(bay) > 0:
            bay_corner = bay[0]
            bay_dim = bay[1]
            bay_centroid = bay[2]            

            if len(bay_corner) == 2  and len(bay_dim) == 2 and len(bay_centroid) == 2:
                # Contour
                img = cv2.rectangle(img, (bay_corner[0], bay_corner[1]), 
                                    (bay_corner[0]+bay_dim[0], bay_corner[1]+bay_dim[1]),
                                      (0, 255, 0), 2)
                # Bearing
                bay_bearing = round(self.object_b.GetBearing(bay_centroid[0]), 1)
                img = cv2.putText(img, str(bay_bearing), 
                                  (bay_centroid[0], bay_centroid[1]), 
                                    cv2.FONT_HERSHEY_SIMPLEX,  
                                    0.5, (0, 0, 0))
                img = cv2.circle(img, (bay_centroid[0], bay_centroid[1]), 5, (0, 255, 0), 5)
                packingBayBearing = bay_bearing

#------------------------------------------------------------------------------------------------
        # Create an ostacle detection object
        obstacles_c = ObstacleContour()
        obstacle_contours, obstacles = obstacles_c.GetContour(hsv_frame)
        obstacle_d = Distance(self.obstacle_height, self.obstacle_width, FOCAL_LENGTH)

        for obstacle in obstacles:
            obstacle_corner = obstacle[0]
            obstacle_dim = obstacle[1]
            obstacle_centroid = obstacle[2]            

            if len(obstacle_corner) == 2  and len(obstacle_dim) == 2 and len(obstacle_centroid) == 2:
                # Contour
                img = cv2.rectangle(img, (obstacle_corner[0], obstacle_corner[1]), 
                                    (obstacle_corner[0]+obstacle_dim[0], obstacle_corner[1]+obstacle_dim[1]),
                                      (0, 255, 0), 2)
                # Bearing
                obstacle_bearing = round(self.object_b.GetBearing(obstacle_centroid[0]), 1)
                img = cv2.putText(img, str(obstacle_bearing), 
                                  (obstacle_centroid[0], obstacle_centroid[1]), 
                                    cv2.FONT_HERSHEY_SIMPLEX,  
                                    0.5, (0, 0, 0))
                img = cv2.circle(img, (obstacle_centroid[0], obstacle_centroid[1]), 5, (0, 255, 0), 5)
                # Distance
                obstacle_distance = round(obstacle_d.GetDistance(obstacle_dim[0], obstacle_dim[1]), 2)
                img = cv2.putText(img, str(obstacle_distance), 
                                  (obstacle_centroid[0], obstacle_centroid[1]-20), 
                                    cv2.FONT_HERSHEY_SIMPLEX,  
                                    0.5, (0, 255, 0))
                obstaclesRangeBearing.append( [obstacle_distance, obstacle_bearing] )
                
#--------------------------------------------------------------------------------------------------
# Create an shelf detection object
        shelf_c = ShelfContour()
        shelf_contours, shelves = shelf_c.GetContour(hsv_frame)

        for shelf in shelves:
            shelf_corner = shelf[0]
            shelf_dim = shelf[1]
            shelf_centroid = shelf[2]            

            if len(shelf_corner) == 2  and len(shelf_dim) == 2 and len(shelf_centroid) == 2:
                # Contour
                img = cv2.rectangle(img, (shelf_corner[0], shelf_corner[1]), 
                                    (shelf_corner[0]+shelf_dim[0], shelf_corner[1]+shelf_dim[1]),
                                      (0, 255, 0), 2)
                # Bearing
                shelf_bearing = round(self.object_b.GetBearing(shelf_centroid[0]), 1)
                img = cv2.putText(img, str(shelf_bearing), 
                                  (shelf_centroid[0], shelf_centroid[1]), 
                                    cv2.FONT_HERSHEY_SIMPLEX,  
                                    0.5, (0, 0, 0))
                img = cv2.circle(img, (shelf_centroid[0], shelf_centroid[1]), 5, (0, 255, 0), 5)
                shelfBearing.append(shelf_bearing)

#------------------------------------------------------------------------------------------------
        if len(shelfBearing) > 0:
            # Create an items detection object
            items_c = ItemContour()
            item_contours, items = items_c.GetContour(hsv_frame)

            for item in items:
                item_corner = item[0]
                item_dim = item[1]
                item_centroid = item[2]

                if len(item_corner) == 2  and len(item_dim) == 2 and len(item_centroid) == 2:
                    # Contour
                    img = cv2.rectangle(img, (item_corner[0], item_corner[1]), 
                                        (item_corner[0]+item_dim[0], item_corner[1]+item_dim[1]),
                                        (0, 255, 0), 2)
                    
                    # Bearing
                    item_bearing = round(self.object_b.GetBearing(item_centroid[0]), 1)
                    img = cv2.putText(img, str(item_bearing), 
                                    (item_centroid[0], item_centroid[1]), 
                                        cv2.FONT_HERSHEY_SIMPLEX,  
                                        0.5, (0, 0, 0))
                    img = cv2.circle(img, (item_centroid[0], item_centroid[1]), 5, (0, 255, 0), 5)
                    itemBearing.append(item_bearing)

#------------------------------------------------------------------------------------------------
        # Show output frame
        cv2.imshow("Camera", img)

        fps = 1.0/(time.time() - t1)         # calculate frame rate
        print("Frame Rate: ", int(fps), end="\r")

        return itemBearing, obstaclesRangeBearing, packingBayBearing, bayMarkerRangeBearing, rowMarkerRangeBearing, shelfBearing
        
                
    def Dispose(self):
        self.cap.close()

    def __frameConfig(self, rgb_frame):
        # Rotate 180deg 
        rgb_frame = cv2.rotate(rgb_frame, cv2.ROTATE_180)

        # Convert the RGB to HSV
        hsv_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2HSV)

        return hsv_frame, rgb_frame

