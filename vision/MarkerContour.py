import cv2
import numpy as np
from scipy.spatial import distance

class MarkerContour:

    lower_bound = np.array([0, 0, 0])
    upper_bound = np.array([179, 54, 50])

    mask = None

    def __init__(self):
        pass
    
    def GetContour(self, hsv_frame):
        
        # Apply Gaussian blur and morphological operations
        blurred_frame = cv2.GaussianBlur(hsv_frame, (5, 5), 0)
        self.mask = cv2.inRange(blurred_frame, self.lower_bound, self.upper_bound)      

        # Filter mask and find contours
        contours = self.__processMask()

        circle_centers = []
        bay_marker = []
        row_marker = []

        # Detect both circles and square
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)

            # Circles
            if perimeter > 0: 
                circularity = 4 * np.pi * (area / (perimeter ** 2))
                if 0.4 < circularity < 1.3:  # Circular threshold for aisle markers
                    # Fit a minimum enclosing circle
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    radius = int(radius)
                    circle_centers.append(center)

                else:
                # If not circular, check for square aspect ratio
                    x, y, w, h = cv2.boundingRect(contour)
                    cX, cY = self.__calculateCentroid(contour)
                    bay_marker = [ [x, y], [w, h], [cX, cY] ]

        if len(circle_centers) != 0:
            # Group the circles by proximity
            circle_groups = self.__groupCircles(circle_centers, threshold=150) 

            # Label aisles based on the number of circles in each group
            for i, group in enumerate(circle_groups, start=1):
                if len(group) == 1:
                    aisle_label = 0
                elif len(group) == 2:
                    aisle_label = 1
                elif len(group) == 3:
                    aisle_label = 2
                else:
                    aisle_label = 3

                #Calculate the average position to display the label
                avg_x = int(sum([c[0] for c in group]) / len(group))
                avg_y = int(sum([c[1] for c in group]) / len(group))
                row_marker = [aisle_label, radius, avg_x, avg_y]

        return contours, bay_marker, row_marker

    
    def __processMask(self):        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask_clean = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, kernel)
        mask_processed = cv2.morphologyEx(mask_clean, cv2.MORPH_OPEN, kernel)

        # Find contours in the processed mask
        contours, _ = cv2.findContours(mask_processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        return contours
    
    def __groupCircles(self, centers, threshold=100):
        groups = []
        for center in centers:
            added = False
            for group in groups:
                # Check if the center is close to any other center in the group
                if any(distance.euclidean(center, c) < threshold for c in group):
                    group.append(center)
                    added = True
                    break
            if not added:
                groups.append([center])
        return groups
    
    def __calculateCentroid(self, contour):
        # Calculate moments of the contour
        M = cv2.moments(contour)

        # Calculate the x, y centroid if M['m00'] is not zero (to avoid division by zero)
        if M['m00'] != 0:
            cX = int(M['m10'] / M['m00'])  # x-coordinate of the centroid
            cY = int(M['m01'] / M['m00'])  # y-coordinate of the centroid
        else:
            cX, cY = 0, 0

        return cX, cY
    
