/*
 * lite_serial_node.hpp
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
#pragma once

#include "lite_serial.hpp"
#include <ros/ros.h>
#include <signal.h>

class LiteSerialNode : public lite_serial::LiteSerial {
	public:
		// Perform rate
		uint32_t rate_;

		// Buff to send
    	uint8_t tx_buff_[128];

		// Buff to receive
    	uint8_t rx_buff_[128];

		// Counter for Tx, Rx
		uint32_t tx_count_;
		uint32_t rx_count_ok_;
		uint32_t rx_count_error_;

		// Time counter for tx, rx
		double tx_elapsed_;
		double rx_elapsed_ok_;
		double rx_elapsed_error_;

		// Open Serial
		void OpenSerial(void);

		// Close Serial
		void CloseSerial(void);

		// Send binary data to serial port
		void TransferData(void);

		// Read binary data from serial port
		void ReceiveData(void);

		// Constructor
		LiteSerialNode();
		
		// Destructor
		~LiteSerialNode();

	private:
		// CRC Polynomal, Left shift
		const uint16_t polynom_ =  0x1021;

		// CRC-CCITT16 table 
		uint16_t crc_table_[256];

		// CRC value of tx_buff and correct rx_buff
		uint16_t crc16_;

		// Record time stamp
		double middle_time_;

		// Create CRC table
		void CreateCrcTable(void);

		// Calculate CRC value
		uint16_t CalculateCrc(uint8_t *data, size_t len);

}; // class LiteSerialNode
// lite_serial_node.hpp