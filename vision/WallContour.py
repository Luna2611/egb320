import cv2
import numpy as np

class WallContour:

    scale_factor = 0
    mask = 0 

    lower_bound = np.array([48, 0, 191])
    upper_bound = np.array([124, 20, 227])


    def __init__(self, scale_factor):
        self.scale_factor = scale_factor

    def GetEdge(self, hsv_frame):

        blurred_frame = cv2.GaussianBlur(hsv_frame, (5, 5), 0) 
        self.mask = cv2.inRange(blurred_frame, self.lower_bound, self.upper_bound)
        
        wall_edges = []  
        contours = []
        wall_area = 0
        range = None

        # Filter mask and find contours
        contours = self.__processMask()

        # Convertanny edge detection on the thresholded image
        edges = cv2.Canny(self.mask, 50, 150)

        # Apply Hough Line Transform
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

        for contour in contours:
            area = cv2.contourArea(contour)
            wall_area += area

        if (wall_area > 40000*self.scale_factor):
            range = "near"
        elif (wall_area > 0): 
            range = "far"
        
        if lines is not None:
            for line in lines:
                    x1, y1, x2, y2 = line[0]
                    wall_edges.append( [ [x1, y1], [x2, y2] ] ) 
                                    
        return wall_edges, range
    
    def __processMask(self): 
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask_clean = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, kernel)
        mask_processed = cv2.morphologyEx(mask_clean, cv2.MORPH_OPEN, kernel)

        # Find contours in the processed mask
        contours, _ = cv2.findContours(mask_processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        return contours
