import cv2
import numpy as np
from scipy.spatial import distance

class MarkerContour:

    lower_bound = np.array([0, 0, 0])
    upper_bound = np.array([179, 79, 66])

    mask = None
    scale_factor = 0

    def __init__(self, scale_factor):
        self.scale_factor = scale_factor
    
    def GetContour(self, hsv_frame):
        # Apply Gaussian blur and morphological operations
        blurred_frame = cv2.GaussianBlur(hsv_frame, (5, 5), 0) 
        self.mask = cv2.inRange(blurred_frame, self.lower_bound, self.upper_bound)
        cv2.imshow("Marker", self.mask)           

        # Filter mask and find contours
        contours = self.__processMask()

        circles = []
        bay_marker = []
        row_marker = []

        # Detect circles
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)

            # Circles
            if perimeter > 0 and area > 0.2*self.scale_factor: 
                circularity = 4 * np.pi * (area / (perimeter ** 2))
                
                if 0.8 < circularity < 1: # Circular threshold for aisle markers
                    # Fit a minimum enclosing circle
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    radius = int(radius)
                    circles.append([center, radius])
                    
        if len(circles) != 0:
            # Group the circles by proximity
            circle_groups = self.__groupCircles(circles, threshold=1000*self.scale_factor) 

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

                # Calculate the average position and radius for each group
                avg_x, avg_y, avg_radius = self.__calculateGroupAverage(group)

                # Store the result for row marker detection
                row_marker = [aisle_label, avg_radius, avg_x, avg_y]

        # Detect squares
        if contours:
            bay_marker_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(bay_marker_contour)
            cX, cY = self.__calculateCentroid(bay_marker_contour)
            bay_marker = [ [x, y] , [w, h], [cX, cY] ]

        return circles, bay_marker, row_marker

    
    def __processMask(self): 
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask_clean = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, kernel)
        mask_processed = cv2.morphologyEx(mask_clean, cv2.MORPH_OPEN, kernel)

        # Find contours in the processed mask
        contours, _ = cv2.findContours(mask_processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        return contours
    
    def __groupCircles(self, circles, threshold=100):
        groups = []
        for circle in circles:
            added = False
            center = circle[0]
            for group in groups:
                if any(distance.euclidean(center, c[0]) < threshold for c in group):
                    group.append(circle)  # Store full circle (center, radius)
                    added = True
                    break
            if not added:
                groups.append([circle])
        return groups

    def __calculateGroupAverage(self, group):
        """
        Calculates the average position and radius for a group of circles.

        Parameters:
        ----------
        group : list
            A list of circles in the group (each represented by [center, radius]).

        Returns:
        -------
        tuple
            The average x, y position and radius for the group.
        """
        num_circles = len(group)

        # Calculate the average x, y coordinates and radius
        avg_x = int(sum([circle[0][0] for circle in group]) / num_circles)
        avg_y = int(sum([circle[0][1] for circle in group]) / num_circles)
        avg_radius = int(sum([circle[1] for circle in group]) / num_circles)

        return avg_x, avg_y, avg_radius

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

    def __getCorners(self, contour):
        """
        Extracts and returns the corners of the marker using cv2.minAreaRect.

        Parameters
        ----------
        contour : np.array
            The contour of the marker.

        Returns
        -------
        np.array or None:
            List of four corner points if contour is found, None otherwise.
        """
        if contour is not None:
            # Get the minimum area rotated rectangle for the contour
            rect = cv2.minAreaRect(contour)

            # Get the four corner points of the rotated rectangle
            box_points = cv2.boxPoints(rect)
            box_points = np.int0(box_points)  # Convert to integer points

            return box_points  # Return the four corner points as a list

        return None  # If no valid contour is found
    
