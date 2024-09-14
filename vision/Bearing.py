import math

class Bearing:
    def __init__(self, image_width, horizontal_fov, focalLength):
        """
        Constructs all the necessary attributes.

        Parameters
        ----------
            focalLength : float
                Apparent focal length of the camera in pixels.
            opticalCenter_x : int
                The optical center in the x-axis of the camera, can be found using image_width/2
        """

        self.center_x = image_width/2
        self.angle_per_px = horizontal_fov/image_width
        self.focal_length = focalLength
    
    # returns distance from camera largest contour
    def GetBearing(self, cx):
        # Use the arctangent formula to calculate the angle
        #theta = math.atan((cx - self.center_x) / self.focal_length)
        # Convert the angle to degrees (optional: depending on your use case)
        #theta_deg = math.degrees(self.__wrapToPi(theta))

        # Pixel offset from center
        px_offset = cx - self.center_x
        theta_deg = px_offset * self.angle_per_px

        return theta_deg
    
    def __wrapToPi(self, radians):
        return ((radians + math.pi) % (2* math.pi) - math.pi)