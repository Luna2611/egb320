import cv2
import numpy as np

class Distance:

    fitted_height = 0.0
    fitted_width = 0.0
    known_width = 0.0
    known_height = 0.0
    focal_length = 0.0

    def __init__(self, knownHeight, knownWidth, focalLength):
        """
        Constructs all the necessary attributes.

        Parameters
        ----------
            focalLength : float
                Apparent focal length of the camera in pixels.
            knownWidth : float
                Width of the vision target in mm
            knownHeight : float
                Height of the vision target in mm
        """

        self.known_width = knownWidth
        self.known_height = knownHeight
        self.focal_length = focalLength
    
    # returns distance from camera largest contour
    def __calculateDistance(self, scale):
        if(self.known_width > 0):
            return round(scale*(self.known_width * self.focal_length) / self.fitted_width, 2)
        else:
            return 0
    
    def getFittedBox(self):
        return self.fitted_height, self.fitted_width
    
    def GetDistance(self, w, h, scale=1):
        '''Returns two frames tracking a vision target of HSV values.
           Updates variables related to vision target tracking'''
        if w <= 0 or h <= 0:
            return 0
        
        self.fitted_height = h
        self.fitted_width = w

        # Prevent width and height in image plane from swapping by checking real width and height
        if (self.known_height > self.known_width and self.fitted_width > self.fitted_height) or \
                (self.known_width > self.known_height and self.fitted_height > self.fitted_width):
            self.fitted_width, self.fitted_height = self.fitted_height, self.fitted_width

        distance = self.__calculateDistance(scale)
        return distance

    def RectifyContour(self, image, corners):
        """
           Applies perspective rectification based on object's corners.

           Parameters
           ----------
           image : np.array
               The input image from the camera.
           corners : list
               List of four corner points of the marker.

           Returns
           -------
           np.array
               The warped image (rectified marker).
        """
        if len(corners) == 4:  # Ensure we have four corner points
            src_pts = np.array(corners, dtype="float32")

            # Define the destination points (a perfect top-down view)
            dst_pts = np.array([[0, 0], [self.known_width, 0],
                                [self.known_width, self.known_height], [0, self.known_height]], dtype="float32")

            # Compute the perspective transform matrix
            mat = cv2.getPerspectiveTransform(src_pts, dst_pts)            

            # Warp the image to a top-down view of the marker
            warped = cv2.warpPerspective(image, mat, (int(self.known_width), int(self.known_height)))

            print("Bay marker: ", warped)

            return warped
        else:
            return None
