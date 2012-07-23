/*
 *  SerialComm.cpp
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

#include "SerialComm.h"

SerialComm::SerialComm() {

}

SerialComm::~SerialComm() {

}

int SerialComm::connect(char *path) {

    int err = 0;
    speed_t baud_rate = B115200;

    // open the USB stream
    fd = open(path, O_RDWR | O_NONBLOCK | O_NDELAY);
   //puts(strerror(errno));

    if(fd==-1){
        return -1;
    }
    // get the baud rate and the flags
    struct termios term;
    tcgetattr(fd, &term);

    err = cfsetispeed(&term, baud_rate);
    if(err != 0) {
        std::cerr << "error setting input speed" << std::endl;
    }
    err = cfsetospeed(&term, baud_rate);
    if(err != 0) {
        std::cerr << "error setting output speed" << std::endl;
    }

    term.c_iflag = 0;
    term.c_oflag = 0;
    term.c_cflag = CLOCAL | CREAD | CS8;
    term.c_lflag = 0;
    term.c_cc[ VTIME ] = 0; //timeout in tenths of a second
    term.c_cc[ VMIN ] = 0;  //no wait for any char

    if( tcsetattr(fd, TCSAFLUSH, &term) < 0)	//termios flags setting
    {
        perror("tcsetattr: ");
    }

    return 0;
}

void SerialComm::disconnect() {
	tcflush(fd,TCIOFLUSH);
	close(fd);
}

void SerialComm::flush() {
	tcflush(fd,TCIOFLUSH);
}

int SerialComm::writeData(char *buf, int num_bytes, int usleep_time) {
	tcflush(fd,TCIOFLUSH);				//flush the input and output buffers before writing new data/commands
	int bytes_written=0;
	int current_bytes_written=0;
	errno=0;
	int timeout = usleep_time/70;		//wait 70 useconds between every read; the loop lasts for usleep_time/70 times
	while(bytes_written<num_bytes) {
		current_bytes_written=write(fd, &buf[bytes_written], num_bytes-bytes_written);
		if(current_bytes_written>=0) {	//if write ok
			bytes_written+=current_bytes_written;
		} //else {
			//FILE *fp = fopen("write_error.txt", "a");
			//fprintf(fp, "Error writing to device: %s\n", strerror(errno));
			//fclose(fp);
			//errno=0;
		//}
		if(bytes_written<num_bytes) {
			usleep(70);				//1/(115200/8) = 70 useconds per byte
			timeout--;
		}
		if(timeout==0) {
			break;
		}
	}
	return bytes_written;
}

int SerialComm::readData(char *buf, int num_bytes, int usleep_time) {
	int bytes_red=0;
	int current_bytes_red=0;
	errno=0;
	int timeout = usleep_time/70;		//wait 70 useconds between every read; the loop lasts for usleep_time/70 times
	while(bytes_red<num_bytes) {
		current_bytes_red=read(fd,&buf[bytes_red],num_bytes-bytes_red);
		if(current_bytes_red>=0) {	//if read ok
			bytes_red+=current_bytes_red;
		} //else {
			//FILE *fp = fopen("read_error.txt", "a");
			//fprintf(fp, "Error reading from device: %s (code %d)\n", strerror(errno), errno);
			//fclose(fp);
			//errno=0;
		//}
		if(bytes_red<num_bytes) {	//wait to receive something new
			usleep(70);				//1/(115200/8) = 70 useconds per byte
			timeout--;
		}
		if(timeout==0) {
			break;
		}		
	}	
	return bytes_red;
}

void SerialComm::discard(int num_bytes) {
	char *temp = (char *)malloc(num_bytes*sizeof(char));
	int bytes_red=0;
	int current_bytes_red=0;
	errno=0;
	while(bytes_red<num_bytes) {
		current_bytes_red=read(fd,&temp[bytes_red],num_bytes-bytes_red);
		if(current_bytes_red>=0) {	//if read ok
			bytes_red+=current_bytes_red;
		} //else {
			//FILE *fp = fopen("discard_error.txt", "a");
			//fprintf(fp, "Error discarding data: %s\n", strerror(errno));
			//fclose(fp);
			//errno=0;
		//}		
	}
	free(temp);
	return;
}
