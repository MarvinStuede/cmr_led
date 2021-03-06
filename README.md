# **cmr_led**: Driver for LED-Panel
<img src="demonstration.gif" width="600">

This repository contains the driver for Sobi's LED Panel and strips. The code for the panel relies on the [rpi-rgb-led-matrix library](https://github.com/hzeller/rpi-rgb-led-matrix) to set the GPIO pins from a ROS Node.
In Sobi, this repository is cloned and executed on the Raspberry pi which is connected with the LED Panel.
The master branch of this repository contains the Code for the panel. The branch `LED-Strips` contains the Arduino Sketch to set the color of the LED Strips in the arms and ears based on ROS messages.

### Prerequisites and Installing
See general instructions [here](https://marvinstuede.github.io/Sobi/software/).
Note that you need to clone this repository with `--recursive` option to install the [rpi-rgb-led-matrix library](https://github.com/hzeller/rpi-rgb-led-matrix)
