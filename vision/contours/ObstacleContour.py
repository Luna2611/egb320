import cv2
import numpy as np

class ObstacleContour:

    lower_bound = np.array([30, 77, 0])
    upper_bound = np.array([47, 180, 78])

    mask = None
    scale_factor = 0    

    def __init__(self, scale_factor):
        self.scale_factor = scale_factor
    
    def GetContour(self, hsv_frame):

        # Apply Gaussian blur to reduce noise
        blurred_frame = cv2.GaussianBlur(hsv_frame, (5, 5), 0)        
        self.mask = cv2.inRange(blurred_frame, self.lower_bound, self.upper_bound)

        # Find contours in the combined mask (to handle obscured obstacles)
        contours = self.__processMask()

        # Detect Obstable
        obstacles_coor = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if (area > 700*self.scale_factor):  # Adjust this threshold based on obstacle size
                # Draw bounding box around each detected obstacle
                corners = self.__getCorners(contour)
                cX, cY = self.__calculateCentroid(contour)
                obstacles_coor.append([ corners, [cX, cY] ])

        return contours, obstacles_coor

    # Function to process a mask and find contours            
    def __processMask(self):        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask_clean = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, kernel)
        mask_processed = cv2.morphologyEx(mask_clean, cv2.MORPH_OPEN, kernel)

        # Find contours in the processed mask
        contours, _ = cv2.findContours(mask_processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        return contours
    
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
    
