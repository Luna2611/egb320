import cv2
import numpy as np

class ObstacleContour:

    lower_bound = np.array([32, 44, 49])
    upper_bound = np.array([50, 255, 255])

    mask = None    

    def __init__(self):
        self.mask = None
    
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
            if area > 200:  # Adjust this threshold based on obstacle size
                # Draw bounding box around each detected obstacle
                x, y, w, h = cv2.boundingRect(contour)
                cX, cY = self.__calculateCentroid(contour)
                obstacles_coor.append([ [x, y] , [w, h], [cX, cY] ])

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
    
