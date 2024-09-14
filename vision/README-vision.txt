
### Contents
This vision folder contains all the objects for image segmentation, distance and bearing detection.
A good place to start is main_vision_example.py which includes examples of how to iterate through all output from Vision object.

### Vision Object
+ SetupCamera() : None
+ Run() : itemsBearing, obstaclesRangeBearing, packingBayBearing, bayMarkerRangeBearing, rowMarkersRangeBearing, shelfBearing
+ Dispose() : None

### Function Explains
* Vision object needs to be initialised before entering a while loop.
* SetupCamera() needs to be called before entering the wile lopp to initialise raspi camerav2
* Calling Run() will calulate the bearing and/or range of 6 objects for ONE FRAME. This function must be called inside the while loop.
* Calling Dispose() at the end to close camera and destroy all graphic windows.

### Output Explains
* itemsBearing: [n] list containing the bearning of n items appear in frame
* obstaclesRangeBearing: [n] list containing [range, bearing] of n obstacle appears in frame
* packingBayBearing : .1f representing the bearing of the yellow packing bay
* bayMarkerRangeBearing: [range, bearing] of the bay marker
* rowMarkersRangeBearing: [3x3] list containing the [row_number, range, bearing] of the row marker. This list is contructed similarly to Copellia Sim, i.e. if camera
			can only detect row 3 marker, the list will be [None, None, [row_number, range, bearing]]
shelfBearing: [n] list containing the bearning of n shelfs appear in frame

### NOTE
- If you want to see the camera view, you could either:
	+ Run main_vision_example.py in RealVNC, or
	+ Run main_vision_example.py through ssh and enable X11Forwarding

- If you want to stop the program while running main_vision_example.py with graphic, click on the graphic window and hit ESC.
- If you want to stop the program while running main_vision_example.py without graphic, hit Ctrl + C in the cmd.