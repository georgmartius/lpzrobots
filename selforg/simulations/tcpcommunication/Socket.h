/*************************************************************************
 *                                                                       *
 * This file is part of Yet Another Robot Simulator (YARS).              *
 * Copyright (C) 2003-2006 Keyan Zahedi and Arndt von Twickel.           *
 * All rights reserved.                                                  *
 * Email: {keyan,twickel}@users.sourceforge.net                          *
 * Web: http://sourceforge.net/projects/yars                             *
 *                                                                       *
 * For a list of contributors see the file AUTHORS.                      *
 *                                                                       *
 * YARS is free software; you can redistribute it and/or modify it under *
 * the terms of the GNU General Public License as published by the Free  *
 * Software Foundation; either version 2 of the License, or (at your     *
 * option) any later version.                                            *
 *                                                                       *
 * YARS is distributed in the hope that it will be useful, but WITHOUT   *
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or *
 * FITNESS FOR A PARTICULAR PURPOSE.                                     *
 *                                                                       *
 * You should have received a copy of the GNU General Public License     *
 * along with YARS in the file COPYING; if not, write to the Free        *
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor,               *
 * Boston, MA 02110-1301, USA                                            *
 *                                                                       *
 *************************************************************************/
 
 
 
#ifndef __SOCKET_COMMUNICATION_H__
#define __SOCKET_COMMUNICATION_H__

#define __YARS_STANDARD_PORT 4500

#include "YarsException.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <string>
#include <vector>

using namespace std;

class Buffer : public vector<char>
{
  public:
    char label;
    // d = double
    // i = int
    // s = string
    // D = vector of double
    // I = vector of int
    //
    // in case of D, I, s the first sizeof(int) bytes give the size of the
    // vector or the length of the string
};


/** \brief TCP/IP Socket communication.
 *
 * This class implements the basic TCP/IP socket communication for clients and
 * for the yars server in C++.
 *
 * This is the low-level communication, i.e. data-types are communicated but not
 * interpreted. These data types are:
 *
 * \li Integer
 * \li Double
 * \li String
 * \li std::vector<int>
 * \li std::vector<double>
 *
 * The interface is the same for all the data types above. Example for a server
 * is:
 *
 \verbatim
 Socket *socket = new Socket();
 socket << 1.2;
 delete socket; \endverbatim
 * 
 * Communication is coded in little endian, i.e. for integers and floating
 * points number, the low byte is send first.
 **/
class Socket
{
  public:
    /** \brief Standard constructor.
     *
     *  Sets the default values. 
     **/
    Socket();

    /** \brief Standard destructor.
     *
     *  Closes sockets, if they are still open.
     **/
    ~Socket();

    /** \brief Client function to open a communication.
     *
     * Open a TCP/IP communication to a server. Most common, the communication
     * with be locally to yars, i.e. on a single machine. Therefore, the
     * standard host is the localhost (127.0.0.1) and the standard port is set
     * to yars' standard port of 4500. 
     * \param[in] std::string host
     * \param[in] integer port
     * \exception YarsException with message "Socket::connect: ECHOCLNT: Error creating listening socket.\n"), if the socket could not be opened
     * \exception YarsException with message "Socket::connect: gethostbyname, cannot resolve hostname"), if the hostname was not resolved
     * \exception YarsException with message "Socket::connect: invalid address given.", if the given address was invalid
     * \exception YarsException with message "Socket::connect: Error creating listening socket.", if the listening port could not be opened
     * \exception YarsException with message "Socket::connect: Error calling connect()", if the connection could not be initialised
     * \exception YarsException with message "Socket::connect: Error setting setsockopt.", if the options for the socket could not be set
     */
    void connect(const std::string host = "127.0.0.1", const int port = __YARS_STANDARD_PORT) throw (YarsException);


    /** \brief Server function to open a communication.
     *
     * Open a TCP/IP communication for clients to connect to. The default port
     * is set to 4500
     * \param[in] integer port
     * \exception YarsException with message if connection could not be opened
     */
    void accept(const int port = 4500)                                        throw (YarsException);




    /** \brief Send a string.
     *
     * This functions sends a string of any length over the communication port.
     * Strings are automatically partitioned by the TCP/IP stack, if necessary.
     *
     * Packing and unpacking of the data is done by the Socket-class, in the
     * following way:
     *
     * \verbatim
     <type char>
     <int size = number of chars to follow>
     <char 1>
     <char 2>
     <char 3>\endverbatim
     *
     * which, in details, is:
     *
     * \verbatim
     <'s'>
     <int low byte 0>
     <int byte 1>
     <int byte 2>
     <int high byte 3>
     <string byte 1>
     <string byte 2>
     ...\endverbatim
     *
     * \param[in] std::string s, the string to be send
     * \exception YarsException with message if something went wrong
     */
    const Socket& operator<<(const std::string&) const;

    /** \brief Receive a string.
     *
     * This function receives a string of any length from the communication
     * port.
     *
     * Packing and unpacking of the data is done by the Socket-class, in the
     * following way:
     *
     * \verbatim
     <type char>
     <int size = number of chars to follow>
     <char 1>
     <char 2>
     <char 3>
     ...\endverbatim
     *
     * which, in details, is:
     *
     *
     \verbatim
     <'s'>
     <int low byte 0>
     <int byte 1>
     <int byte 2>
     <int high byte 3>
     <string byte 1>
     <string byte 2>
     ... \endverbatim
     * \param[out] std::string s, the received string
     * \exception YarsException with detailed message if the received data type
     * does not match the expected, i.e. is not a string.
     */
    const Socket& operator>>(std::string&) const                              throw(YarsException);




    /** \brief Send a double.
     *
     * This functions sends a double over the communication port. A double is
     * given by 8 bytes. 
     *
     * Packing and unpacking of the data is done by the Socket-class, in the
     * following way:
     *
     * \verbatim
     <type char>
     <double bytes> \endverbatim 
     *
     * which, in details, is:
     *
     * \verbatim
     <'d'>
     <low byte = byte 0>
     <byte 1>
     <byte 2>
     <byte 3>
     <byte 4>
     <byte 5>
     <byte 6>
     <high byte = byte 7>\endverbatim 
     *
     * \param[in] double d, the double to be send
     */
    const Socket& operator<<(const double&) const;


    /** \brief Receives a double.
     *
     * This functions receives a double over the communication port. A double is
     * given by 8 bytes. 
     *
     * Packing and unpacking of the data is done by the Socket-class, in the
     * following way:
     *
     * \verbatim
     <type char>
     <double bytes> \endverbatim 
     *
     * which, in details, is:
     *
     * \verbatim
     <'d'>
     <low byte = byte 0>
     <byte 1>
     <byte 2>
     <byte 3>
     <byte 4>
     <byte 5>
     <byte 6>
     <high byte = byte 7>\endverbatim 
     *
     * \param[out] double d, the double to be send
     * \exception YarsException with detailed message if the received data type
     * does not match the expected, i.e. is not a double.
     */
    const Socket& operator>>(double&) const                                   throw(YarsException);





    /** \brief Send an int.
     *
     * This functions sends an int over the communication port. A int is
     * given by 4 bytes. 
     *
     * Packing and unpacking of the data is done by the Socket-class, in the
     * following way:
     *
     * \verbatim
     <type char>
     <int bytes> \endverbatim 
     *
     * which, in details, is:
     *
     * \verbatim
     <'i'>
     <low byte = byte 0>
     <byte 1>
     <byte 2>
     <high byte = byte 3>\endverbatim 
     *
     * \param[in] int i, the integer to be send
     */
    const Socket& operator<<(const int&) const;

    /** \brief Receives an int.
     *
     * This functions receives an int over the communication port. A int is
     * given by 4 bytes. 
     *
     * Packing and unpacking of the data is done by the Socket-class, in the
     * following way:
     *
     * \verbatim
     <type char>
     <int bytes> \endverbatim 
     *
     * which, in details, is:
     *
     * \verbatim <'i'> <low byte = byte 0> <byte 1> <byte 2> <high byte = byte 3>\endverbatim 
     *
     * \param[out] int i, the integer to be send
     * \exception YarsException with detailed message if the received data type
     * does not match the expected, i.e. is not a double.
     */
    const Socket& operator>>(int&) const                                      throw(YarsException);

    /** \brief Send a vector of integers.
     *
     * This functions sends several integers of the communication port, which
     * are given to the function in form of a std::vector<int>. Each int is
     * given by 4 bytes. 
     *
     * Packing and unpacking of the data is done by the Socket-class, in the
     * following way:
     *
     * \verbatim
     <type char>
     <vector size bytes>
     <first integer bytes>
     <second integer bytes> ...\endverbatim 
     *
     * which, in details, is:
     *
     * \verbatim
     <'I'>
     <size low byte = byte 0>
     <byte 1>
     <byte 2>
     <size high byte = byte 3>
     <first integer low byte = byte 0>
     <first integer byte 1>
     <first integer byte 2>
     <first integer high byte = byte 3>
     <second integer bytes> ... \endverbatim
     *
     * \param[in] std::vector<int> vi, the vector of integers to be send
     */
    const Socket& operator<<(const vector<int>&) const;

    /** \brief Receives a vector of integers.
     *
     * This functions receives several integers of the communication port, which
     * are stored by the function in form of a std::vector<int>. Each int is
     * given by 4 bytes. 
     *
     * Packing and unpacking of the data is done by the Socket-class, in the
     * following way:
     *
     * \verbatim
     <type char>
     <vector size bytes>
     <first integer bytes>
     <second integer bytes> ...\endverbatim 
     *
     * which, in details, is:
     *
     * \verbatim
     <'I'>
     <size low byte = byte 0>
     <byte 1>
     <byte 2>
     <size high byte = byte 3>
     <first integer low byte = byte 0>
     <first integer byte 1>
     <first integer byte 2>
     <first integer high byte = byte 3>
     <second integer bytes> ... \endverbatim
     *
     * \param[out] std::vector<int> vi, the vector of integers to be receives
     * \exception YarsException with detailed message if the received data type
     * does not match the expected, i.e. is not a double.
     */
    const Socket& operator>>(vector<int>&) const                              throw(YarsException);

    /** \brief Send a vector of doubles.
     *
     * This functions sends several doubles of the communication port, which
     * are given to the function in form of a std::vector<double>. Each double is
     * given by 8 bytes. 
     *
     * Packing and unpacking of the data is done by the Socket-class, in the
     * following way:
     *
     * \verbatim
     <type char>
     <vector size bytes>
     <first double bytes>
     <second double bytes> ...\endverbatim 
     *
     * which, in details, is:
     *
     * \verbatim
     <'D'>
     <size low byte = byte 0>
     <byte 1>
     <byte 2>
     <size high byte = byte 3>
     <first double low byte = byte 0>
     <first double byte 1>
     <first double byte 2>
     <first double byte 3>
     <first double byte 4>
     <first double byte 5>
     <first double byte 6>
     <first double high byte = byte 7>
     <second doubles bytes> ... \endverbatim
     *
     * \param[in] std::vector<double> vd, the vector of doubles to be send
     */
    const Socket& operator<<(const vector<double>&) const;

    /** \brief Receives a vector of doubles.
     *
     * This functions receives several doubles of the communication port, which
     * are then stored by the function in form of a std::vector<double>. Each double is
     * given by 8 bytes. 
     *
     * Packing and unpacking of the data is done by the Socket-class, in the
     * following way:
     *
     * \verbatim
     <type char>
     <vector size bytes>
     <first double bytes>
     <second double bytes> ...\endverbatim 
     *
     * which, in details, is:
     *
     * \verbatim
     <'D'>
     <size low byte = byte 0>
     <byte 1>
     <byte 2>
     <size high byte = byte 3>
     <first double low byte = byte 0>
     <first double byte 1>
     <first double byte 2>
     <first double byte 3>
     <first double byte 4>
     <first double byte 5>
     <first double byte 6>
     <first double high byte = byte 7>
     <second doubles bytes> ... \endverbatim
     *
     * \param[out] std::vector<doubles> vd, the vector of integers to be received
     * \exception YarsException with detailed message if the received data type
     * does not match the expected, i.e. is not a double.
     */
    const Socket& operator>>(vector<double>&) const                           throw(YarsException);

    void close();

  private:
    const Socket& operator<<(const Buffer&) const;
    const Socket& operator>>(Buffer&) const                                   throw(YarsException);

    void __check(const char a, const char b)                                  throw(YarsException);
    void __readDouble(double *d, Buffer b, int startIndex);
    void __readInteger(int *i, Buffer b, int startIndex);
    void __writeDouble(Buffer *b, double d);
    void __writeInteger(Buffer *b, int i);
    void __coneverToInt(char *c, int *i);

    int                _mysock;
    int                _sock;
    int                _tcpClientSock;
    sockaddr_in        _addr;

    struct sockaddr_in _myself;
    struct sockaddr_in _peer;
};

#endif // __SOCKET_COMMUNICATION_H__
