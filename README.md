# Overview
lite_serial is a light weight C++ serial library developed for ROS, or other system collaborating wiht C++ complier.

It can be used to transfer & receive binary data to/from opposite device being not ROS node.

It works in non-canonical, raw, blocking mode. Tested on UBUNTU 18.04 & ROS melodic, environmwnt is listed as below.


# Test Evironment
ROS master(lite_serial implemented): Lenovo L560 UBUNTU 18.04, ROS melodic

Opposite device: IMU (Microchip SMAG55 implemented)

USB connection: USB2.0 CDC virtual serial com port, Full Speed, 12Mbps


# Demonstration
lite_serial_ros(lite_serial_node) is a experiment package(node) to echo binary data with opposite device with CRC check.
Refer to the following image of test result, another one in non-blocking mode based on pyserial lib writen in python as well.
As you see, almost the same results were obtained. The Rx time in image includes reading/writing one of opposite device
and reading one, CRC check of lite_serial_node. So, the Rx time measured(around 0.7-0.8ms) is longer tha Tx one(0.2-0.3ms).

The pycode(serial_echo_test.py) is uploaded to github as well. Feel free to fork and share.


lite_serial_ros.jpg
![alt text](https://github.com/soarbear/lite_serial_ros/blob/main/image/lite_serial_ros.jpg)

python_serial_test.jpg
![alt text](https://github.com/soarbear/lite_serial_ros/blob/main/image/python_serial_test.jpg)

# Language
More information in Japanese coming soon.
