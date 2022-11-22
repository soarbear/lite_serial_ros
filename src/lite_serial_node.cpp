/*
 * lite_serial_node.cpp
 *
 * License: BSD-3-Clause
 *
 * Copyright (c) 2012-2022 Shoun Corporation <research.robosho@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of RT Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "lite_serial_node.hpp"

/*
 * LiteSerialNode(), LiteSerialNode class constractor
 */
LiteSerialNode::LiteSerialNode() {

    // Initialize counter for tx, rx
    tx_count_ = 0;
    rx_count_ok_ = 0;
    rx_count_error_ = 0;

    // Initialize duration counter for tx, rx
	tx_elapsed_ = 0.0;
    rx_elapsed_ok_ = 0.0;
    rx_elapsed_error_ = 0.0;

    // Iteration rate(Hz)
    rate_ = 500;

	// Initialize buff to send
    for (int32_t count = 0; count < sizeof(tx_buff_); count++) {  
        tx_buff_[count] = count+1;
    }

    // Create CRC table
    CreateCrcTable();

    // Calculate CRC of tx_buff_
    crc16_ = CalculateCrc(tx_buff_, sizeof(tx_buff_));
}

/*
 * ~LiteSerialNode(), LiteSerialNode class destructor
 */
LiteSerialNode::~LiteSerialNode() {
    // Add close/free if needed
}

/*
 * OpenSerial(), Open serial port, binary raw data, non-canonical, blocking mode
 */
void LiteSerialNode::OpenSerial() {
    // Try connecting serial port
    while (ros::ok()) {
        if (LiteSerial::Open()) { // Connected
            ROS_INFO_ONCE("[Open] %s opened as binary data, non-canonical, raw, blocking mode", port_name_.c_str());
            return;
        }
        else { // Disconnected
            ROS_INFO_ONCE("[Open] Opening %s, it takes over several seconds", port_name_.c_str());
            ros::Duration(1.0).sleep(); // Sleep for 0.5s, try modifying to shorter or longer value according to needs
        }
    }
}

/*
 * OpenSerial(), Close serial port
 */
void LiteSerialNode::CloseSerial() {
//
}

/*
 * TransferData(), Transfer binary data, non-canonical, raw, blocking mode
 */
void LiteSerialNode::TransferData() {

    // Record starting time
    double start_time = ros::Time::now().toSec();

    // Send buffer data to imu
    size_t tx_size = LiteSerial::WriteBuff(tx_buff_, sizeof(tx_buff_));

    // Wait until data be sent
    tcdrain(serial_fd_);

    // Calculte elapsed time
    middle_time_ = ros::Time::now().toSec();
    double elapsed = middle_time_- start_time;
    tx_elapsed_ += elapsed;
    tx_count_++;
    ROS_INFO("[Tx] transfered: %dbytes, counted: %utimes, duration: %.3lfms, average duration: %.3lfms, crc_tx: 0x%04x", 
            (int32_t)tx_size, tx_count_, elapsed*1000.0, tx_elapsed_/tx_count_*1000.0, crc16_);
}

/*
 * ReceiveData(), Receive binary data, non-canonical, raw, blocking mode
 */
void LiteSerialNode::ReceiveData() {
    // Read data from serial port
    if (LiteSerial::ReadBuff(rx_buff_, sizeof(rx_buff_)) < sizeof(rx_buff_)) { // Disconnected
        ROS_ERROR_ONCE("[Rx] USB disconnected, press Ctrl+c to stop the node, roslaunch again");
        while (ros::ok());
    }
    else { // Reading be ok
        // CRC check
        uint16_t crc16_rx = CalculateCrc(rx_buff_, sizeof(rx_buff_));
        if (crc16_rx == crc16_) { // CRC passed
            double elapsed = ros::Time::now().toSec() - middle_time_;
            rx_elapsed_ok_ += elapsed;
            rx_count_ok_++;
            ROS_INFO("[Rx] CRC passed, received: %lu-bytes, counted: %utimes, duration: %.3lfms, average duration: %.3lfms, crc_rx: 0x%04x(passed)\n",
                    sizeof(rx_buff_), rx_count_ok_, elapsed*1000.0, rx_elapsed_ok_/rx_count_ok_*1000.0, crc16_rx);
        }
        else { // CRC error once
            double elapsed = ros::Time::now().toSec() - middle_time_;
            rx_elapsed_error_ += elapsed;
            rx_count_error_++;
            ROS_WARN("[Rx] CRC error, received: %lubytes, counted: %utimes, duration: %.3lfms, average duration: %.3lfms, crc_rx: 0x%04x(error)\n", 
                    sizeof(rx_buff_), rx_count_error_, elapsed*1000.0, rx_elapsed_error_/rx_count_error_*1000.0, crc16_rx);
        }
    }
}

/*
 * CreateCrcTable(), RC-CCITT16 Table, Left shift
 */
void LiteSerialNode::CreateCrcTable(void) {
    int32_t i, j;
    uint16_t crc;
    for (i = 0; i < 256; i++) {
        crc = (i << 8);
        for (j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = polynom_ ^ (crc << 1);
            }
            else {
                crc <<= 1;
            }
        }
        crc_table_[i] = crc;
    }
}

/*
 * Crc16Table(), Calculate CRC-CCITT16, Left shift
 */
uint16_t LiteSerialNode::CalculateCrc(uint8_t *data, size_t size) {
    uint32_t crc = 0xffff, final = 0x0000;
    uint32_t temp;

    for (uint32_t i = 0; i < size; ++i) {
        temp = (*data++ ^ (crc >> 8)) & 0xff;
        crc = crc_table_[temp] ^ (crc << 8);
    }

    return static_cast<uint16_t>(crc ^ final);
}

// Serial port file descriptor
int32_t serial_fd = -1;

// Struct for config restoration
struct termios tio_back;

/*
 * sigint_handler(), Ctrl+c handler
 */
void sigint_handler(int sig) {
    if (serial_fd >= 0) {
        // Flush input & output buffer
        tcflush(serial_fd, TCIFLUSH);
        tcflush(serial_fd, TCOFLUSH);

        // Store configuration
        tcsetattr(serial_fd, TCSANOW, &tio_back);

        // Release fd
        close(serial_fd);
    }
    ros::shutdown();
}

/*
 * main(), lite_serial_node, echo between ROS(lite_serial_node) and opposite device not being ROS(node) throuth USB serial port
 */
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "lite_serial_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");

    // Create an instance
    LiteSerialNode serial_node;

    // Get parameters from config file
    nh.getParam("serial_port_name", serial_node.port_name_);

    // Open serial port
    serial_node.OpenSerial();
    serial_fd = serial_node.GetSerialFd();
    tio_back = serial_node.GetTermio();

    // Set Ctrl+c handler
    signal(SIGINT, sigint_handler);

    // Set rate(Hz)
    ros::Rate rate(serial_node.rate_);

    // Iteration to perform echo with opposite MCU/CPU
    while (ros::ok()) {
        
        // Transfer binary data
        serial_node.TransferData();

        // Receive binary data
        serial_node.ReceiveData();
        
        // Sleep a while according to rate_
        rate.sleep();
    }

    // Shut down
    serial_node.CloseSerial();
    ROS_INFO("[Close] Tx: %lubytes, counted: %utimes, average Tx_duration: %.3lfms; Rx_CRC_passed: %lubytes, counted: %utimes, average Rx_duration(incl.CRC): %.3lfms; Rx_CRC_error(incl.CRC): %lubytes, counted: %utimes, average Rx_duration: %.3lfms\n",
            sizeof(serial_node.tx_buff_), serial_node.tx_count_, serial_node.tx_elapsed_/serial_node.tx_count_*1000.0, sizeof(serial_node.rx_buff_), serial_node.rx_count_ok_, serial_node.rx_elapsed_ok_/serial_node.rx_count_ok_*1000.0, 
            sizeof(serial_node.rx_buff_), serial_node.rx_count_error_, serial_node.rx_elapsed_error_/(serial_node.rx_count_error_|0x1)*1000.0);
    ROS_INFO("[Close] lite_serial_node done");
    return 0;
}
// lite_serial_node.cpp