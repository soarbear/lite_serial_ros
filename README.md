# Overview
lite_serial is a light weight C++ serial library developed for ROS, or other systems collaborating with C++ complier (note: modification of termio needed ).

lite_serial(lite_serial.hpp & lite_serial.cpp) can be used to transfer / receive binary data to / from opposite device being not ROS node. Using roslaunch as follows to run the lite_serial_node.

$ roslaunch lite_serial_ros lite_serial_demo.launch

lite_serial works in non-canonical, raw, blocking mode. Tested on UBUNTU 18.04 & ROS melodic, environmwnt is listed as below.

# Test Evironment
ROS master(lite_serial implemented): Lenovo L560 (UBUNTU 18.04, ROS melodic implemented)

Opposite device: IMU (Microchip SMAG55 implemented, ROS unimplemented, non-ROS node)

USB connection: USB2.0 (CDC virtual serial com port, Full Speed, 12Mbps)


# Demonstration

lite_serial_ros(lite_serial_node) is a experiment package(node) to echo binary data with opposite device, performing CRC check to Rx data.
At the same time, as a comparison we run another prgram with non-blocking mode based on pyserial library.
As a result of the comparison, almost the same results were obtained. The Rx time includes reading/writing one of opposite device, reading one of lite_serial_ros, CRC check of lite_serial_node. So, the Rx time(around 0.7-0.8ms) is longer than Tx one(0.2-0.3ms).

The python code(python_serial_test.py) is uploaded to github as well. 

blocking mode, lite_serial_ros.jpg
![alt text](https://github.com/soarbear/lite_serial_ros/blob/main/image/lite_serial_ros.jpg)

non-blocking mode(in python), python_serial_test.jpg
![alt text](https://github.com/soarbear/lite_serial_ros/blob/main/image/python_serial_test.jpg)

# TODO
More information in Japanese coming soon.
Feel free to fork and share the repository.
