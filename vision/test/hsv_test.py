import cv2
import picamera2
import numpy as np
import time

FRAME_WIDTH = 820
FRAME_HEIGHT = 616
SCALE_FACTOR = 0.2
#320, 240
#640, 480

FOCAL_LENGTH = 69 #px
#636

HORIZONTAL_FOV = 51.5 #deg

class Vision:
    
    # Row marker measurements
    marker_radius = 70 #mm

    # Rubic measurements
    #known_width = 38 #mm
    #known_height = 38 #mm

    # Obstacle measurements
    obstacle_width = 50 #mm
    obstacle_height = 150 #mm    

    cap = None

    def SetupCamera(self):
        # Create a camera object
        self.cap = picamera2.Picamera2()
        self.cap.set_controls({"AeEnable": False})  # Disable Auto Exposure
        self.cap.set_controls({"ExposureTime": 5000, "AnalogueGain": 0.8})  # Set manual values
        self.cap.set_controls({"FrameDurationLimits": (10000, 15000)})  # Set min to 10ms and max to 15 ms for a fixed frame rate
        
        config = self.cap.create_video_configuration(main={"format":'RGB888',"size":(FRAME_WIDTH, FRAME_HEIGHT)})
        self.cap.configure(config)

        #start the camera
        self.cap.start()

#print the different camera resolutions/modes 
#the sensor can be configured for
print("Sensor Mode: ", cap.sensor_modes)

# Initial HSV GUI slider values to load on program start
iVal = (0, 0, 0, 179, 255, 255)

cv2.namedWindow('colorTest', cv2.WINDOW_AUTOSIZE)
# Lower range colour sliders.
cv2.createTrackbar('lowHue', 'colorTest', iVal[0], 179, nothing)
cv2.createTrackbar('lowSat', 'colorTest', iVal[1], 255, nothing)
cv2.createTrackbar('lowVal', 'colorTest', iVal[2], 255, nothing)
# Higher range colour sliders.
cv2.createTrackbar('highHue', 'colorTest', iVal[3], 179, nothing)
cv2.createTrackbar('highSat', 'colorTest', iVal[4], 255, nothing)
cv2.createTrackbar('highVal', 'colorTest', iVal[5], 255, nothing)

#set a specific configuration, smaller resolution will be faster
#however will have a cropped field of view
#consider a balance between higher resolution, field of view and frame rate
#config = cap.create_video_configuration(main={"format":'XRGB8888',"size":(FRAME_WIDTH, FRAME_HEIGHT)})
config = cap.create_video_configuration(main={"format":'RGB888',"size":(FRAME_WIDTH, FRAME_HEIGHT)})
cap.configure(config)

#start the camera
cap.start()

while(1):
    t1 = time.time()                     # for measuring fps

    # Get HSV value from the sliders
    lowHue = cv2.getTrackbarPos('lowHue', 'colorTest')
    lowSat = cv2.getTrackbarPos('lowSat', 'colorTest')
    lowVal = cv2.getTrackbarPos('lowVal', 'colorTest')
    highHue = cv2.getTrackbarPos('highHue', 'colorTest')
    highSat = cv2.getTrackbarPos('highSat', 'colorTest')
    highVal = cv2.getTrackbarPos('highVal', 'colorTest')
    
    frame = cap.capture_array()          # capture a single image frame

    frame = cv2.rotate(frame, cv2.ROTATE_180)

    # Convert the XRGB to HSV
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    # HSV values to define a colour range we want to create a mask from
    colorLow = np.array([lowHue, lowSat, lowVal])
    colorHigh = np.array([highHue, highSat, highVal])
    mask = cv2.inRange(frameHSV, colorLow, colorHigh)

    # Show the first mask
    cv2.imshow('mask-plain', mask)

    # Find contours
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
    if len(contour_sizes) > 0:
         
        biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]

        x, y, w, h = cv2.boundingRect(biggest_contour)
        cv2.rectangle(frame, (x,y), (x+w, y+h), (0, 255, 0), 2)

    # Show output frame
    cv2.imshow("hsvTest", frame)

    k = cv2.waitKey(5) & 0xFF   # Make the program wait for 5ms before continuing (also required to display image).
    if k == 27: # Esc key
        break

    fps = 1.0/(time.time() - t1)         # calculate frame rate
    print("Frame Rate: ", int(fps), end="\r")

cap.close()
cv2.destroyAllWindows()

