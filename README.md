# Blinky #
Blinky is an experimental Raspberry Pi project to use OpenCV and a Waveshare Alphabot to pick up all the small things. 


Blinky is made with the following hardware:
* Raspberry Pi 3
* Waveshare Alphabot
* Raspberry Pi Camera 2
* 7.4V LiPo Battery
* Servos that I found at my house

## Object Tracking ##
The object tracking system takes a still frame from the camera and filters out all pixels that don't fall within a 
specified color range. Then it performs an erosion and dilation to smooth out the results. 

Once all matching pixels have been found, OpenCV's contour tool can match the pixels to a circular area and return its center.



## Resources ##
* https://docs.opencv.org/master/d9/df8/tutorial_root.html
* https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/

A Raspberry Pi Camera 2 uses OpenCv to search for yellow pixels, and the pixel location is converted into a distance 
and angle relative to the bot.