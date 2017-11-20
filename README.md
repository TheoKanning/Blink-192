# Blink-192 #
Blink-192 is an experimental Raspberry Pi project to use [OpenCV](https://opencv.org/) and a Waveshare AlphaBot to pick up all the small things. 


Blink-192 is made with the following hardware:
* Raspberry Pi 3
* Waveshare AlphaBot - I bought [this kit](https://www.amazon.com/gp/product/B01N1JWFKZ/ref=oh_aui_detailpage_o09_s00?ie=UTF8&psc=1) without a Pi Included
* Raspberry Pi Camera 2
* 7.4V LiPo Battery - I bought [this one](https://www.amazon.com/gp/product/B06Y2M2J7D/ref=oh_aui_detailpage_o03_s01?ie=UTF8&psc=1)
* Servos that I found at my house

## Installation ##
The following tools must be installed on the Raspberry Pi.
* Raspbian (Another OS would probably work, but why make things difficult?)
* OpenCV - [Linux installation instructions](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html)
* Picamera - [Instructions](https://picamera.readthedocs.io/en/release-1.13/install.html)

## Usage ##

Run the following command to get started.

``` sudo python blink192.py```

If you want to customize how the color recognition works, `ball_tracker_test.py` has some useful debugging options.

* `--debug` will show an extra frame with the results of the masking operations as well as the HSV values of the center pixel.
* `--low` and `--high` allow you to set the low and high threshold from the command line

For example:

``` python ball_tracker_test.py --debug --low 25 150 150```


## Object Tracking ##
The object tracking system takes a still frame from the camera and filters out all pixels that don't fall within a 
specified color range. Then it performs an erosion and dilation to smooth out the results. 

Once all matching pixels have been found, OpenCV's contour tool can match the pixels to a circular area and return its center.

## State Management ##
The Blink-192 system is a simple state machine with the following states:
1. Searching - Move around randomly until a ball is found
2. Moving to Ball - Move towards ball
3. Picking Up Ball - Moving arm to pick up ball

If the robot loses track of the ball, it will return to the Searching state. After attempting to pick up a ball, the 
robot will return to the Searching state. There is currently no way to know if a pick-up attempt was successful.

## Resources ##
* https://docs.opencv.org/master/d9/df8/tutorial_root.html
* https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/