[blink-192]: ./blink-192.jpg

# Blink-192 #
Blink-192 is an experimental Raspberry Pi project that uses [OpenCV](https://opencv.org/) and a Waveshare AlphaBot to pick up all the small things. 

![Blink 192][blink-192]

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

## Obstacle Detection ##
The WaveShare Alphabot comes with two infrared detectors on the front. If one of the detectors find an object, the robot will turn the opposite way, and if both see an object it will go in reverse.

## Operating Logic ##
The Blink-192 system is currently stateless, and follows this simple logic.
1. If an obstacle is detected, turn or go in reverse to avoid
2. If a ball is seen, turn towards it and move forward
3. Move forward

Blink-192 currently does not create a map of known objects, but this simple logic is enough to randomly explore environments and avoid obstacles.

## Resources ##
* https://docs.opencv.org/master/d9/df8/tutorial_root.html
* https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
