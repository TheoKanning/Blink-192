[blink-192]: ./pictures/blink-192.jpg
[graph]: ./pictures/rosgraph.png

![Blink 192][blink-192]

# Blink-192 #
Blink-192 is an experimental Rapsberry Pi robot that uses ROS to move and stream video.

Heavily based on the Teleop-bot example from Programming Robots with ROS [O'Reilly Media](http://shop.oreilly.com/product/0636920024736.do)

## History ##
I named this robot Blink-192 because it has a lot of bright lights, and its local ip address starts with 192. 

Originally, this robot used OpenCV to track and pick up all the small things around the office, but later I modified it to learn ROS.
The `opencv` branch still has the original implementation (https://github.com/TheoKanning/Blink-192/tree/opencv).

## Hardware ##
Blink-192 is made with the following hardware:
* Raspberry Pi 3
* Waveshare AlphaBot - I bought [this kit](https://www.amazon.com/gp/product/B01N1JWFKZ/ref=oh_aui_detailpage_o09_s00?ie=UTF8&psc=1) without a Pi Included
* Raspberry Pi Camera 2
* 7.4V LiPo Battery - I bought [this one](https://www.amazon.com/gp/product/B06Y2M2J7D/ref=oh_aui_detailpage_o03_s01?ie=UTF8&psc=1)

## Software ##
* Raspberry Pi Ubuntu image - [Ubiquity Robotics](https://downloads.ubiquityrobotics.com/pi.html)
* Pi Camera Node - [Ubiquity Robotics](https://github.com/UbiquityRobotics/raspicam_node)

Note that this particular Ubuntu image isn't required, but it comes with ROS pre-installed and creates a wifi access point automatically. 

## ROS Package Structure ##

![graph][graph]

### Nodes ###
* `keyboard_driver`: Reads keystrokes and publishes them to the `keys` topic
* `keys_to_twist`: Converts key messages into `Twist` commands and publishes them to the `cmd_vel` topic
* `motors`: Subscribes to `cmd_vel` and controls motors
* `raspicam_node`: Publishes video stream to `/raspicam_node/image`

### Topics ###
* `/keys`: `String` containing latest keystroke
* `/cmd_vel`: `Twist` containing desired linear and angular velocity
* `/raspicam_node/image`: `sensor_msgs/Image` from Raspberry Pi Camera

### Rviz ###
* `config.rviz`: Configures rviz to show `/raspicam_node/image` full-screen

### Launch Files ###
* `desktop.launch`: Runs `keyboard_driver` node and rviz
* `robot.launch`: Runs `keys_to_twist`, `motors`, and `raspicam_node`
* `run-local.launch`: Runs everything

## Resources ##
* http://shop.oreilly.com/product/0636920024736.do
* https://learn.ubiquityrobotics.com/
* https://www.youtube.com/watch?v=9Ht5RZpzPqw
