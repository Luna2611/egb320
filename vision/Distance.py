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
                Width of the vision target in units
            knownHeight : float
                Height of the vision target in units
        """

        self.known_width = knownWidth
        self.known_height = knownHeight
        self.focal_length = focalLength
    
    # returns distance from camera largest contour
    def __calculateDistance(self):
        if(self.known_width > 0):
            return round((self.known_width * self.focal_length) / self.fitted_width, 2)
        else:
            return 0
    
    def getFittedBox(self):
        return self.fitted_height, self.fitted_width
    
    def GetDistance(self, w, h):
        '''Returns two frames tracking a vision target of HSV values.
           Updates variables related to vision target tracking'''
        if w <= 0 or h <= 0:
            return 0
        
        self.fitted_height = h
        self.fitted_width = w

        # Prevent width and height in image plane from swapping by checking real width and height
        if(self.known_height > self.known_width and self.fitted_width > self.fitted_height):
            temp = self.fitted_width
            self.fitted_width = self.fitted_height
            self.fitted_height = temp
        elif(self.known_width > self.known_height and self.fitted_height > self.fitted_width):
                temp = self.fitted_height
                self.fitted_height = self.fitted_width
                self.fitted_width = temp

        distance = self.__calculateDistance()
        return distance
