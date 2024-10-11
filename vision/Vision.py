import cv2
import picamera2
import numpy as np
import time

from position.Distance import Distance
from position.Bearing import Bearing

from contours.MarkerContour import MarkerContour
from contours.ItemContour import ItemContour
from contours.ObstacleContour import ObstacleContour
from contours.ShelfContour import ShelfContour
from contours.BayContour import BayContour
from contours.WallContour import WallContour

FRAME_WIDTH = 820
FRAME_HEIGHT = 616
SCALE_FACTOR = 0.2
#320, 240
#640, 480

FOCAL_LENGTH = 69 #px
#636

HORIZONTAL_FOV = 51.5 #deg

class Vision:
    
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
        self.object_b = Bearing(FRAME_WIDTH*SCALE_FACTOR, HORIZONTAL_FOV, FOCAL_LENGTH)

    def SetupCamera(self):
        # Create a camera object
        self.cap = picamera2.Picamera2()
        self.cap.controls.ExposureTime = 5000
        self.cap.controls.AnalogueGain = 0.8
        
        config = self.cap.create_video_configuration(main={"format":'RGB888',"size":(FRAME_WIDTH, FRAME_HEIGHT)})
        self.cap.configure(config)

        #start the camera
        self.cap.start()
    
    # Process frame from video feed, keep this in a while loop
    def Run(self): 

        itemBearing = []
        packingBayBearing = None
        obstaclesRangeBearing = []
        rowMarkerRangeBearing = [None,None,None]
        shelfBearing = []
        bayMarkerRangeBearing = []
        wallRange = 0

        t1 = time.time()                     # for measuring fps
      
        flipped_rgb_frame = self.cap.capture_array()          # capture a single image frame
        hsv_frame, rgb_frame = self.__frameConfig(flipped_rgb_frame)
        img = rgb_frame

#------------------------------------------------------------------------------------------------
        # Create an wall detection object
        wall_c = WallContour(SCALE_FACTOR)
        wall_edges, wallRange = wall_c.GetEdge(hsv_frame)

        for edge in wall_edges:
            begin_point = edge[0]
            print()
            end_point = edge[1]
            img = cv2.line(img, (begin_point[0], begin_point[1]), (end_point[0], end_point[1]), (212, 54, 149), 2)               

#------------------------------------------------------------------------------------------------
        # Create an bay detection object
        bay_c = BayContour(SCALE_FACTOR)
        bay_contour, bay = bay_c.GetContour(hsv_frame)

        if len(bay) > 0:
            bay_corner = bay[0]
            bay_dim = bay[1]
            bay_centroid = bay[2]            

            if len(bay_corner) == 2  and len(bay_dim) == 2 and len(bay_centroid) == 2:
                # Contour
                img = cv2.rectangle(img, (bay_corner[0], bay_corner[1]), 
                                    (bay_corner[0]+bay_dim[0], bay_corner[1]+bay_dim[1]),
                                      (46, 232, 187), 2)
                # Bearing
                bay_bearing = round(self.object_b.GetBearing(bay_centroid[0]), 1)
                img = cv2.putText(img, str(bay_bearing), 
                                  (bay_centroid[0], bay_centroid[1]), 
                                    cv2.FONT_HERSHEY_SIMPLEX,  
                                    0.5, (0, 0, 0))
                img = cv2.circle(img, (bay_centroid[0], bay_centroid[1]), 5, (0, 255, 0), 5)
                packingBayBearing = bay_bearing

#------------------------------------------------------------------------------------------------
        # Create a marker detection object
        marker_c = MarkerContour(SCALE_FACTOR)
        circles, bay_marker, row_markers = marker_c.GetContour(hsv_frame)

        marker_d = Distance(self.marker_radius, self.marker_radius , FOCAL_LENGTH)        
            
        if len(row_markers) > 0 and packingBayBearing is None:
            # Contour 
            for circle in circles:
                img = cv2.circle(img, circle[0], circle[1], (255, 0, 0) , 2) 

            # Bearing
            row_marker_bearing = round(self.object_b.GetBearing(row_markers[2]), 1)
            img = cv2.putText(img, str(row_marker_bearing), 
                                (row_markers[2] + 10, row_markers[3]), 
                                cv2.FONT_HERSHEY_SIMPLEX,  
                                0.5, (0, 0, 0))
            img = cv2.circle(img, (row_markers[2], row_markers[3]), 5, (0, 255, 0), 5)

            # Distance
            row_marker_d = round(marker_d.GetDistance(row_markers[1], row_markers[1]), 2)
            if (row_marker_d > 900): 
                row_marker_d -= row_marker_d*0.23
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

        elif len(bay_marker) > 0 and packingBayBearing is not None:
            bay_marker_corner = bay_marker[0]
            bay_marker_dim = bay_marker[1]
            bay_marker_centroid = bay_marker[2]

            # Contour
            img = cv2.rectangle(img, (bay_marker_corner[0], bay_marker_corner[1]), 
                                (bay_marker_corner[0]+bay_marker_dim[0], bay_marker_corner[1]+bay_marker_dim[1]),
                                (255, 0, 0), 2)            
                     
            # Bearing
            bay_marker_bearing = round(self.object_b.GetBearing(bay_marker_centroid[0]), 1)
            img = cv2.putText(img, str(bay_marker_bearing), 
                                (bay_marker_centroid[0], bay_marker_centroid[1]), 
                                cv2.FONT_HERSHEY_SIMPLEX,  
                                0.5, (0, 0, 0))
            img = cv2.circle(img, (bay_marker_centroid[0], bay_marker_centroid[1]), 5, (0, 255, 0), 5)

            # Distance
            bay_marker_d = round(marker_d.GetDistance(bay_marker_dim[0], bay_marker_dim[1]), 2) + 40
            bay_marker_d += (bay_marker_d*0.3 + 200)
            img = cv2.putText(img, str(bay_marker_d), 
                                  (bay_marker_centroid[0], bay_marker_centroid[1]-20), 
                                    cv2.FONT_HERSHEY_SIMPLEX,  
                                    0.5, (0, 255, 0))
            bayMarkerRangeBearing =  [bay_marker_d, bay_marker_bearing]

#------------------------------------------------------------------------------------------------
        # Create an ostacle detection object
        obstacles_c = ObstacleContour(SCALE_FACTOR)
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
                                      (255, 126, 0), 2)
                # Bearing
                obstacle_bearing = round(self.object_b.GetBearing(obstacle_centroid[0]), 1)
                img = cv2.putText(img, str(obstacle_bearing), 
                                  (obstacle_centroid[0], obstacle_centroid[1]), 
                                    cv2.FONT_HERSHEY_SIMPLEX,  
                                    0.5, (0, 0, 0))
                img = cv2.circle(img, (obstacle_centroid[0], obstacle_centroid[1]), 5, (0, 255, 0), 5)
                # Distance
                obstacle_distance = round(obstacle_d.GetDistance(obstacle_dim[0], obstacle_dim[1]), 2)
                obstacle_distance += obstacle_distance * 0.75
                img = cv2.putText(img, str(obstacle_distance), 
                                  (obstacle_centroid[0], obstacle_centroid[1]-20), 
                                    cv2.FONT_HERSHEY_SIMPLEX,  
                                    0.5, (0, 255, 0))
                obstaclesRangeBearing.append( [obstacle_distance, obstacle_bearing] )
                
#--------------------------------------------------------------------------------------------------
# Create an shelf detection object
        shelf_c = ShelfContour(SCALE_FACTOR)
        shelf_areas, shelves = shelf_c.GetContour(hsv_frame)

        for shelf in shelves:
            shelf_corner = shelf[0]
            shelf_dim = shelf[1]
            shelf_centroid = shelf[2]            

            if len(shelf_corner) == 2  and len(shelf_dim) == 2 and len(shelf_centroid) == 2:
                # Contour
                img = cv2.rectangle(img, (shelf_corner[0], shelf_corner[1]), 
                                    (shelf_corner[0]+shelf_dim[0], shelf_corner[1]+shelf_dim[1]),
                                      (142, 235, 32), 2)
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
            items_c = ItemContour(SCALE_FACTOR)
            item_contours, items = items_c.GetContour(hsv_frame)

            for item in items:
                item_corner = item[0]
                item_dim = item[1]
                item_centroid = item[2]

                if len(item_corner) == 2  and len(item_dim) == 2 and len(item_centroid) == 2:
                    # Contour
                    img = cv2.rectangle(img, (item_corner[0], item_corner[1]), 
                                        (item_corner[0]+item_dim[0], item_corner[1]+item_dim[1]),
                                        (224, 19, 108), 2)
                    
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
        print("FUUUUUUCK " + str(rowMarkerRangeBearing))
        return itemBearing, obstaclesRangeBearing, packingBayBearing, bayMarkerRangeBearing, rowMarkerRangeBearing, shelfBearing, wallRange
        
                
    def Dispose(self):
        self.cap.close()

    def __frameConfig(self, rgb_frame):
        # Rotate 180deg 
        rgb_frame = cv2.rotate(rgb_frame, cv2.ROTATE_180)
        scale_down_frame = cv2.resize(rgb_frame, None, fx= SCALE_FACTOR, fy= SCALE_FACTOR, interpolation= cv2.INTER_AREA)

        # Convert the RGB to HSV
        hsv_frame = cv2.cvtColor(scale_down_frame, cv2.COLOR_RGB2HSV)

        return hsv_frame, scale_down_frame

