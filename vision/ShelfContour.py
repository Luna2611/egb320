import cv2
import numpy as np

class ShelfContour:

    lower_bound = np.array([0, 108, 24])
    upper_bound = np.array([20, 255, 255])

    mask = None
    scale_factor = 0    

    def __init__(self, scale_factor):
        self.scale_factor = scale_factor
    
    def GetContour(self, hsv_frame):
        
        # Apply Gaussian blur and morphological operations
        blurred_frame = cv2.GaussianBlur(hsv_frame, (5, 5), 0)
        self.mask = cv2.inRange(blurred_frame, self.lower_bound, self.upper_bound)      

        # Find contours in the combined mask (to handle obscured shelf)
        combined_contours = self.__processMask()

        # Detect Obstable
        shelfs_coor = []
        areas = []
        for contour in combined_contours:
            area = cv2.contourArea(contour)
            if area > 9000*self.scale_factor:  # Adjust this threshold based on shelf size
                # Draw bounding box around each detected shelf
                x, y, w, h = cv2.boundingRect(contour)
                cX, cY = self.__calculateCentroid(contour)
                shelfs_coor.append([ [x, y] , [w, h], [cX, cY] ])
                areas.append(area)

        return areas, shelfs_coor

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
    
    # Function to apply Sobel edge detection
    def __applySobel(self):
        # Apply Sobel edge detection in both x and y directions
        sobel_x = cv2.Sobel(self.mask, cv2.CV_64F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(self.mask, cv2.CV_64F, 0, 1, ksize=3)
        
        # Calculate the gradient magnitude
        sobel_combined = cv2.magnitude(sobel_x, sobel_y)
        
        # Normalize to the range 0-255 and convert to uint8
        sobel_combined = cv2.convertScaleAbs(sobel_combined)
        
        # Apply a binary threshold to the Sobel output to get a binary edge map
        _, sobel_binary = cv2.threshold(sobel_combined, 50, 255, cv2.THRESH_BINARY)
        
        return sobel_binary
