import picamera2
import cv2
import numpy as np
import math

def onClick(event, x, y, flags, params):
	global image_points
	global focal_length
	if event == cv2.EVENT_LBUTTONDOWN:
		print(x,y)
		if not found_focalLength:
			image_points.append([x,y])
		else:
			print(focal_length)
			
	
def calculateDistance(x1, y1, x2, y2):
        dist = math.sqrt((x2 - x1)**2 + (y2-y1)**2)
        return dist
			
def findFocalLength(known_distance, known_height, pixel_height):
	focalLength = (pixel_height * known_distance) / known_height
	return focalLength
	
def findPixelWidth(frame, lower, upper):
	fittedWidth = 0
	hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

	mask = cv2.inRange(hsv, lower, upper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	
	_, thresh = cv2.threshold(mask, 30, 255, cv2.THRESH_BINARY)
	contours = cv2.findContours(thresh, cv2.RETR_LIST,  cv2.CHAIN_APPROX_SIMPLE)[-2]
	
	x = 0
	y = 0
	w = 0
	h = 0
	
	if(len(contours) > 0):
		c = max(contours, key=cv2.contourArea)
		x, y, w, h = cv2.boundingRect(c)
		rect = cv2.minAreaRect(c)
		box = cv2.boxPoints(rect)
		box = np.int0(box)
		cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)        
		fittedHeight = calculateDistance(box[0][0], box[0][1], box[1][0], box[1][1])
		
	return c, mask, fittedHeight
	

# click the points in the image (order is important) to populate the image points
image_points = []

# intialise the camera
cap = picamera2.Picamera2()

# add camera configuration settings here
config = cap.create_video_configuration(main={"format":'RGB888',"size":(640, 480)})
cap.configure(config)
cap.start()

# capture
frame = cap.capture_array()

# reduce size to speed up	
frame = cv2.rotate(frame, cv2.ROTATE_180)

# display
"""cv2.imshow("Image", frame)
cv2.setMouseCallback("Image", onClick)"""

# click four points and then fit the homography
found_focalLength = False
known_distance = 300
known_width = 70

# Orange hsv range
lower_orange = np.array([41, 139, 155])
upper_orange = np.array([139, 255, 255])

# Black hsv range
lower_black = np.array([0, 0, 0])
upper_black = np.array([179, 54, 50])

while True:
	k = cv2.waitKey(5) & 0xFF   # Make the program wait for 5ms before continuing (also required to display image).
	if k == 27: # Esc key
		break

	if not found_focalLength:
		c, mask, height_in_rf_image = findPixelWidth(frame, lower_black, upper_black)
		print("Found Width: ", height_in_rf_image)
		cv2.imshow("Mask", mask)
		cv2.drawContours(frame, c, -1, (0, 255, 0), 3) 
		cv2.imshow("Frame", frame)


		if height_in_rf_image > 0:
			focal_length = findFocalLength(known_distance, known_width, height_in_rf_image)
			found_focalLength = True
			print("found focal length")
			print(focal_length)
					
			
# shutdown
cap.close()
cv2.destroyAllWindows()
