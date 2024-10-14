import cv2
import picamera2
import numpy as np
import time
import math

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
SCALE_FACTOR = 0.25
# 320, 240
# 640, 480

FOCAL_LENGTH = 70 # px
# 636

HORIZONTAL_FOV = 51.5  # deg


class Vision:
    # Row marker measurements
    marker_radius = 70  # mm

    # Obstacle measurements
    obstacle_width = 50  # mm
    obstacle_height = 150  # mm

    cap = None

    # Params for displaying
    font_scale = 0.5
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_color = (0, 0, 0)
    contour_thickness = 2
    centroid_radius = 3

    # Contour color in BGR
    bay_color = (255, 149, 69)  # blue-ish
    shelf_color = (98, 249, 252)  # yellow-ish
    row_marker_color = (32, 109, 255)  # orange
    bay_marker_color = (255, 32, 150)  # purple-ish
    obstacle_color = (32, 32, 255)  # red
    wall_color = (213, 32, 255)  # pink
    item_color = (32, 255, 140)  # green



    def __init__(self):
        self.object_b = Bearing(FRAME_WIDTH * SCALE_FACTOR, HORIZONTAL_FOV, FOCAL_LENGTH)

    def SetupCamera(self):
        # Create a camera object
        self.cap = picamera2.Picamera2()
        self.cap.controls.ExposureTime = 7000
        self.cap.controls.AnalogueGain = 1.1
        
        config = self.cap.create_video_configuration(main={"format":'RGB888',"size":(FRAME_WIDTH, FRAME_HEIGHT)})
        self.cap.configure(config)

        # start the camera
        self.cap.start()

    # Process frame from video feed, keep this in a while loop
    def Run(self):

        itemBearing = []
        packingBayBearing = None
        obstaclesRangeBearing = []
        rowMarkerRangeBearing = [None, None, None]
        shelfBearing = []
        bayMarkerRangeBearing = []
        wallRange = 0

        t1 = time.time()  # for measuring fps

        flipped_rgb_frame = self.cap.capture_array()  # capture a single image frame
        hsv_frame, rgb_frame = self.__frameConfig(flipped_rgb_frame)
        img = rgb_frame

        # Wall ------------------------------------------------------------------------------------------------
        # Create an wall detection object
        wall_c = WallContour(SCALE_FACTOR)
        wall_edges, wallRange = wall_c.GetEdge(hsv_frame)

        for edge in wall_edges:
            begin_point = edge[0]
            end_point = edge[1]
            img = cv2.line(img, (begin_point[0], begin_point[1]), (end_point[0], end_point[1]),
                           self.wall_color, self.contour_thickness)

        # Packing Bay ------------------------------------------------------------------------------------------------
        # Create an bay detection object
        bay_c = BayContour(SCALE_FACTOR)
        bay_contour, bay = bay_c.GetContour(hsv_frame)

        if len(bay) > 0:
            bay_corner = bay[0]
            bay_dim = bay[1]
            bay_centroid = bay[2]

            if len(bay_corner) == 2 and len(bay_dim) == 2 and len(bay_centroid) == 2:
                # Contour
                img = cv2.rectangle(img, (bay_corner[0], bay_corner[1]),
                                    (bay_corner[0] + bay_dim[0], bay_corner[1] + bay_dim[1]),
                                    self.bay_color, self.contour_thickness)
                # Bearing
                bay_bearing = round(self.object_b.GetBearing(bay_centroid[0]), 1)
                img = cv2.putText(img, str(bay_bearing),
                                  (bay_centroid[0], bay_centroid[1]),
                                  self.font, self.font_scale, self.font_color)
                img = cv2.circle(img, (bay_centroid[0], bay_centroid[1]),
                                 self.centroid_radius, self.bay_color, self.centroid_radius)
                packingBayBearing = bay_bearing

        # Markers ------------------------------------------------------------------------------------------------
        # Create a marker detection object
        marker_c = MarkerContour(SCALE_FACTOR)
        circles, bay_marker, row_markers = marker_c.GetContour(hsv_frame)

        marker_d = Distance(self.marker_radius, self.marker_radius, FOCAL_LENGTH)

        if len(circles) > 0 and len(row_markers) > 0 and packingBayBearing is None:
            if row_markers[0] < 3:
                # Contour
                for circle in circles:
                    img = cv2.circle(img, circle[0], circle[1], self.row_marker_color, self.contour_thickness)

                # Bearing
                row_marker_bearing = round(self.object_b.GetBearing(row_markers[2]), 1)
                img = cv2.putText(img, str(row_marker_bearing),
                                  (row_markers[2] + 10, row_markers[3]),
                                  self.font, self.font_scale, self.font_color)
                img = cv2.circle(img, (row_markers[2], row_markers[3]),
                                 self.centroid_radius, self.row_marker_color, self.centroid_radius)

                # Distance
                row_marker_d = marker_d.GetDistance(row_markers[1], row_markers[1])
                """if row_marker_d > 900:
                    row_marker_d -= row_marker_d * 0.23"""
                img = cv2.putText(img, str(row_marker_d),
                                  (row_markers[2], row_markers[3] - 20),
                                  self.font, self.font_scale, self.font_color)
                img = cv2.putText(img, str(row_markers[0] + 1),
                                  (row_markers[2], row_markers[3] + 20),
                                  self.font, self.font_scale, self.font_color)

                index = row_markers[0]
                rowMarkerRangeBearing[index] = [row_markers[0] + 1, row_marker_d, row_marker_bearing]

        elif len(bay_marker) > 0 and packingBayBearing is not None:
            bay_marker_corners = bay_marker[0]
            bay_marker_dim = bay_marker[1]
            bay_marker_centroid = bay_marker[2]

            if len(bay_marker_corners) == 2 and len(bay_marker_dim) == 2 and len(bay_marker_centroid) == 2: 
                # Contour
                img = cv2.rectangle(img, (bay_marker_corners[0], bay_marker_corners[1]),
                                    (bay_marker_corners[0] + bay_marker_dim[0], bay_marker_corners[1] + bay_marker_dim[1]),
                                    self.bay_marker_color, self.contour_thickness)

                # Bearing
                bay_marker_bearing = round(self.object_b.GetBearing(bay_marker_centroid[0]), 1)
                img = cv2.putText(img, str(bay_marker_bearing),
                                    (bay_marker_centroid[0], bay_marker_centroid[1]),
                                    self.font, self.font_scale, self.font_color)
                img = cv2.circle(img, (bay_marker_centroid[0], bay_marker_centroid[1]), 
                                    self.centroid_radius, self.bay_marker_color, self.centroid_radius)

                # Distance
                scale = bay_marker_dim[0]/bay_marker_dim[1]
                bay_marker_d = marker_d.GetDistance(bay_marker_dim[0], bay_marker_dim[1], scale)*2
                img = cv2.putText(img, str(bay_marker_d),
                                    (bay_marker_centroid[0], bay_marker_centroid[1] - 20),
                                    self.font, self.font_scale, self.font_color)
                bayMarkerRangeBearing = [bay_marker_d, bay_marker_bearing]

        # Obstacle ------------------------------------------------------------------------------------------------
        # Create an obstacle detection object
        obstacles_c = ObstacleContour(SCALE_FACTOR)
        obstacle_contours, obstacles = obstacles_c.GetContour(hsv_frame)
        obstacle_d = Distance(self.obstacle_height, self.obstacle_width, FOCAL_LENGTH)

        for obstacle in obstacles:
            obstacle_corners = obstacle[0]
            obstacle_dim = obstacle[1]
            obstacle_centroid = obstacle[2]

            if len(obstacle_corners) == 2 and len(obstacle_centroid) == 2 and len(obstacle_dim) == 2:
                # Contour
                img = cv2.rectangle(img, (obstacle_corners[0], obstacle_corners[1]),
                                    (obstacle_corners[0] + obstacle_dim[0], obstacle_corners[1] + obstacle_dim[1]),
                                    self.obstacle_color, self.contour_thickness)

                # Bearing
                obstacle_bearing = round(self.object_b.GetBearing(obstacle_centroid[0]), 1)
                img = cv2.putText(img, str(obstacle_bearing),
                                    (obstacle_centroid[0], obstacle_centroid[1] + 20),
                                    self.font, self.font_scale, self.font_color)
                img = cv2.circle(img, (obstacle_centroid[0], obstacle_centroid[1]), 
                                    self.centroid_radius, self.obstacle_color, self.centroid_radius)

                # Distance
                scale = 1 #obstacle_dim[0]/obstacle_dim[1]
                obstacle_distance = obstacle_d.GetDistance(obstacle_dim[0], obstacle_dim[1], scale)
                img = cv2.putText(img, str(obstacle_d),
                                    (obstacle_centroid[0], obstacle_centroid[1] - 20),
                                    self.font, self.font_scale, self.font_color)

                obstaclesRangeBearing.append([obstacle_distance, obstacle_bearing])

        # Shelves --------------------------------------------------------------------------------------------------
        # Create an shelf detection object
        shelf_c = ShelfContour(SCALE_FACTOR)
        shelf_areas, shelves = shelf_c.GetContour(hsv_frame)

        for shelf in shelves:
            shelf_corner = shelf[0]
            shelf_dim = shelf[1]
            shelf_centroid = shelf[2]

            if len(shelf_corner) == 2 and len(shelf_dim) == 2 and len(shelf_centroid) == 2:
                # Contour
                img = cv2.rectangle(img, (shelf_corner[0], shelf_corner[1]),
                                    (shelf_corner[0] + shelf_dim[0], shelf_corner[1] + shelf_dim[1]),
                                    self.shelf_color, self.contour_thickness)
                # Bearing
                shelf_bearing = round(self.object_b.GetBearing(shelf_centroid[0]), 1)
                img = cv2.putText(img, str(shelf_bearing),
                                  (shelf_centroid[0], shelf_centroid[1]),
                                  self.font, self.font_scale, self.font_color)
                img = cv2.circle(img, (shelf_centroid[0], shelf_centroid[1]), 5, (0, 255, 0), 5)
                shelfBearing.append(shelf_bearing)

        # Items ------------------------------------------------------------------------------------------------
        if len(shelfBearing) > 0:
            # Create an items detection object
            items_c = ItemContour(SCALE_FACTOR)
            item_contours, items = items_c.GetContour(hsv_frame)

            for item in items:
                item_corner = item[0]
                item_dim = item[1]
                item_centroid = item[2]

                if len(item_corner) == 2 and len(item_dim) == 2 and len(item_centroid) == 2:
                    # Contour
                    img = cv2.rectangle(img, (item_corner[0], item_corner[1]),
                                        (item_corner[0] + item_dim[0], item_corner[1] + item_dim[1]),
                                        self.item_color, self.contour_thickness)

                    # Bearing
                    item_bearing = round(self.object_b.GetBearing(item_centroid[0]), 1)
                    img = cv2.putText(img, str(item_bearing),
                                      (item_centroid[0], item_centroid[1]),
                                      self.font, self.font_scale, self.font_color)
                    img = cv2.circle(img, (item_centroid[0], item_centroid[1]), 5, (0, 255, 0), 5)
                    itemBearing.append(item_bearing)

        # Display ------------------------------------------------------------------------------------------------
        # Show output frame
        cv2.imshow("Camera", img)

        fps = 1.0 / (time.time() - t1)  # calculate frame rate
        print("Frame Rate: ", int(fps), end="\r")
        return itemBearing, obstaclesRangeBearing, packingBayBearing, bayMarkerRangeBearing, rowMarkerRangeBearing, shelfBearing, wallRange

    def Dispose(self):
        self.cap.close()

    def __frameConfig(self, rgb_frame):
        # Rotate 180deg 
        rgb_frame = cv2.rotate(rgb_frame, cv2.ROTATE_180)
        scale_down_frame = cv2.resize(rgb_frame, None, fx=SCALE_FACTOR, fy=SCALE_FACTOR, interpolation=cv2.INTER_AREA)

        # Convert the RGB to HSV
        hsv_frame = cv2.cvtColor(scale_down_frame, cv2.COLOR_RGB2HSV)

        return hsv_frame, scale_down_frame
