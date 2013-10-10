/*
 *  SerialComm.h
 *  EPuckMonitor
 *
 *  Created by Stefano Morgani on 11/18/08.
 *
 *	Copyright 2008 GCtronic
 *
 *  This file is part of EPuckMonitor.
 *
 *  EPuckMonitor is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  EPuckMonitor is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with EPuckMonitor; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 * 
 */

/*! \class SerialComm
 *  \brief Class handling the serial communication under Linux/MacOS systems.
 *  \author Stefano Morgani
 *  \version 1.0
 *  \date 11/18/08
 *
 * This class contains all the functions needed to communicate with the E-Puck robot.
 */

#ifndef SERIALCOMM_H_
#define SERIALCOMM_H_

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>

class SerialComm {

	public:
		SerialComm();
		~SerialComm();
		
		/**
			This function is used to initialize and open the connection to the bluetooth device; the serial settings are: 115200-8-N-1 and no hw/sw control.
			@param	path	string indicating the path of the device to which the connection will be established (e.g. /dev/tty.e-puck_1594-COM1-1).
			@return			returns the file descriptor returned by the "open" function.
		*/
		int connect(char *path);
		
		/**
			This function closes the connection opened.
		*/		
		void disconnect();
		
		/**
			This function is used to send data to the device.
			@param	buf			buffer that contains the characters (command) to be written to the serial port.
			@param	num_bytes	the number of bytes that will be written to the serial port (length of the buffer).
			@param	usleep_time	specifies the total waiting time before exiting the writing operation in case of errors; the value is specified in microseconds unit.
			@return				returns the number of bytes correctly written to the serial port.
		*/		
		int writeData(char *buf, int num_bytes, int usleep_time);

		/**
			This function is used to receive data from the device.
			@param	buf			buffer that will contain the data received from the device.
			@param	num_bytes	the number of bytes to be read from the device (maximum length of the buffer).
			@param	usleep_time	specifies the total waiting time before exiting the reading operation in case of errors; the value is specified in microseconds unit.
			@return				returns the number of bytes correctly red from the device.
		*/
		int readData(char *buf, int num_bytes, int usleep_time);
		
		/**
			This function flushes both the input and output buffers.
		*/		
		void flush();
		
		/**
			This function is used to discard data from the input buffer; note that this function is blocking, that is it will continue to read data from the buffer until all the requested bytes are discarded.
			@param	num_bytes	the number of bytes to be discarded from the input buffer.
		*/		
		void discard(int num_bytes);
		
	private:
		int fd;															/**< file descriptor indicating the bluetooth device*/
};

#endif
