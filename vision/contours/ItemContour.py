import cv2
import numpy as np

class ItemContour:

    lower_bound = np.array([80, 170, 100]) #96
    upper_bound = np.array([150, 255, 255])

    mask = None
    scale_factor = 0    

    def __init__(self, scale_factor):
        self.scale_factor = scale_factor
    
    def GetContour(self, hsv_frame):
        
        # Apply Gaussian blur and morphological operations
        blurred_frame = cv2.GaussianBlur(hsv_frame, (5, 5), 0)
        self.mask = cv2.inRange(blurred_frame, self.lower_bound, self.upper_bound)
        cv2.imshow("Item", self.mask)

        # Filter mask and find contours
        contours = self.__processMask()         

        # Detect items
        items_coor = []
        if contours is not None:
            for contour in contours:
                area = cv2.contourArea(contour)

                if area > 5*self.scale_factor:
                    # Draw bounding box around each detected orange item
                    x, y, w, h = cv2.boundingRect(contour)
                    cX, cY = self.__calculateCentroid(contour)
                    items_coor.append([ [x, y] , [w, h], [cX, cY] ])
        
        return contours, items_coor
                
    def __processMask(self):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask_clean = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, kernel)
        mask_processed = cv2.morphologyEx(mask_clean, cv2.MORPH_OPEN, kernel)

        # Find contours in the processed mask
        _, thresh = cv2.threshold(mask_processed, 30, 255, cv2.THRESH_BINARY)
        contours = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]

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
