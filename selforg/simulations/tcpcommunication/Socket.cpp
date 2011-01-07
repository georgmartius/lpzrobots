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
 
 
#include "Socket.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <sstream>

#define __BUFFER_SIZE    8192 // just required to put the bytes together again. is not a real buffer size

#define __DOUBLE_VALUE   'd'
#define __INTEGER_VALUE  'i'
#define __STRING_VALUE   's'

#define __DOUBLE_VECTOR  'D'
#define __INTEGER_VECTOR 'I'


using namespace std;

inline void __printBytes(char c)
{
  for(int i = 0; i < 8; i++)
  {
    if(0x01 & (c >> i))
    {
      std::cout << "1";
    }
    else
    {
      std::cout << "0";
    }
  }
  std::cout << std::endl;
}
 
Socket::Socket()
{
  _sock = -1;
  _mysock = -1;
}

Socket::~Socket()
{
  close();
}

/** \brief Function for the client
 *
 *  Closes sockets, if they are still open.
 **/
void Socket::connect(const std::string host, const int port) throw (YarsException)
{
  struct hostent *h;
  struct in_addr **addr_list;
  int flag = 1;

  if ( (_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
    throw YarsException("Socket::connect: ECHOCLNT: Error creating listening socket.\n");
  }

  if ((h = gethostbyname(host.c_str())) == NULL) {  // get the host info
    herror("gethostbyname");
    throw YarsException("Socket::connect: gethostbyname, cannot resolve hostname");
  }

  addr_list = (struct in_addr **)h->h_addr_list;

  if ( inet_aton(inet_ntoa(*addr_list[0]), &_peer.sin_addr) <= 0 ) {
    throw YarsException("Socket::connect: invalid address given.");
  }

  memset(&_peer, 0, sizeof(_peer));
  _peer.sin_family      = AF_INET;
  _peer.sin_port        = htons(port);

  if ( (_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
    throw YarsException("Socket::connect: Error creating listening socket.");
  }

  if ( ::connect(_sock, (struct sockaddr *) &_peer, sizeof(_peer) ) < 0 ) {
    perror("Connect error:");
    throw YarsException("Socket::connect: Error calling connect()");
  }
  
  if(setsockopt(_sock, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int)) < 0)
  {
    throw YarsException("Socket::connect: Error setting setsockopt.");
  }
}


void Socket::accept(const int port) throw (YarsException)
{
  int flag    = 1;
  int p = port - 1;
  memset(&_peer, 0, sizeof(_peer));
  _peer.sin_family      = AF_INET;
  _peer.sin_addr.s_addr = htonl(INADDR_ANY);

  if ((_mysock = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
    throw YarsException("Socket::accept ECHOSERV: Error creating listening socket.");
  }

  do {
    p++;
    _peer.sin_port        = htons(p);
  } while(bind(_mysock, (struct sockaddr *) &_peer, sizeof(_peer)) < 0 );

  cout << "  --> on port " << p<< endl;

  if ( listen(_mysock, 1) < 0 ) { // only one connection on this port
    throw YarsException("Socket::accept ECHOSERV: Error calling listen()");
  }

  if ( (_sock = ::accept(_mysock, NULL, NULL) ) < 0 ) { // ::accept != Socket::accept
    throw YarsException("Socket::accept ECHOSERV: Error calling accept()");
  }

  if (setsockopt(_sock, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int)) < 0)
  {
    throw YarsException("Socket::connect: Error setting setsockopt on my socket.");
  }

  if(setsockopt(_mysock, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int)) < 0)
  {
    throw YarsException("Socket::connect: Error setting setsockopt on client socket.");
  }
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// BUFFER 
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// BUFFER - SEND
////////////////////////////////////////////////////////////////////////////////
const Socket& Socket::operator<<(const Buffer &b) const
{
  char buf[b.size() + 1];
  buf[0] = b.label;

  for(unsigned int i = 0; i < b.size(); i++)
  {
    buf[1 + i] = b[i];
  }

  send(_sock, buf, b.size() + 1, 0);

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
// BUFFER - RECEIVE
////////////////////////////////////////////////////////////////////////////////
const Socket& Socket::operator>>(Buffer &b) const throw(YarsException)
{
  b.resize(0);
  char buf[__BUFFER_SIZE];
  char type[1];
  char sizeBytes[4];
  int r        = 0;
  int size     = -1;
  int read     = 0;
  int toread   = 0;

  recv(_sock, type, 1, MSG_WAITALL);
  ((Socket*)this)->__check(b.label, type[0]);

  switch(type[0])
  {
    case __STRING_VALUE:
      recv(_sock, sizeBytes, 4, MSG_WAITALL);
      for(int i = 0; i < 4; i++) b.push_back(sizeBytes[i]);
      ((Socket*)this)->__coneverToInt(sizeBytes, &size);
      break;
    case __INTEGER_VECTOR:
      recv(_sock, sizeBytes, 4, MSG_WAITALL);
      for(int i = 0; i < 4; i++) b.push_back(sizeBytes[i]);
      ((Socket*)this)->__coneverToInt(sizeBytes, &size);
      size *= sizeof(int);
      break;
    case __DOUBLE_VECTOR:
      recv(_sock, sizeBytes, 4, MSG_WAITALL);
      for(int i = 0; i < 4; i++) b.push_back(sizeBytes[i]);
      ((Socket*)this)->__coneverToInt(sizeBytes, &size);
      size *= sizeof(double);
      break;
    case __INTEGER_VALUE:
      size = sizeof(int);
      break;
    case __DOUBLE_VALUE:
      size = sizeof(double);
      break;
  }

  read = 0;
  while(read < size)
  {
    toread = std::min(size - read, __BUFFER_SIZE);
    r = recv(_sock, buf, toread, MSG_WAITALL);
    read += r;
    for(int i = 0; i < r; i++)
    {
      b.push_back(buf[i]);
    }
  }
  return *this;
}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// DOUBLE
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// DOUBLE - SEND
////////////////////////////////////////////////////////////////////////////////
const Socket& Socket::operator<<(const double &d) const
{
  Buffer b;
  b.resize(0);
  b.label = __DOUBLE_VALUE; 
  ((Socket*)this)->__writeDouble(&b, d);
  *this << b;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
// DOUBLE - RECEIVE
////////////////////////////////////////////////////////////////////////////////
const Socket& Socket::operator>>(double& d) const throw(YarsException)
{
  d = 0;
  int size = sizeof(double);
  Buffer b;
  b.resize(size);
  b.label = __DOUBLE_VALUE;
  *this >> b;
  ((Socket*)this)->__readDouble(&d, b, 0);
  return *this;
}





////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// INTEGER 
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// INTEGER - SEND
////////////////////////////////////////////////////////////////////////////////
const Socket& Socket::operator<<(const int &i) const
{
  Buffer b;
  b.label = __INTEGER_VALUE;
  b.resize(0);
  ((Socket*)this)->__writeInteger(&b, i);
  *this << b;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
// INTEGER - RECEIVE
////////////////////////////////////////////////////////////////////////////////
const Socket& Socket::operator>>(int& i) const throw(YarsException)
{
  i = 0;
  int size = sizeof(int);
  Buffer b;
  b.label = __INTEGER_VALUE;
  b.resize(size);
  *this >> b;
  ((Socket*)this)->__readInteger(&i, b, 0);
  return *this;
}





////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// STRING 
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// STRING - SEND
////////////////////////////////////////////////////////////////////////////////
const Socket& Socket::operator<<(const std::string& s) const
{
  Buffer b;
  b.resize(0);
  b.label = __STRING_VALUE;
  ((Socket*)this)->__writeInteger(&b, (int)s.length());
  for(unsigned int i = 0; i < s.length(); i++)
  {
    b.push_back(s[i]);
  }
  *this << b;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
// STRING - RECEIVE
////////////////////////////////////////////////////////////////////////////////
const Socket& Socket::operator>>(std::string& s) const throw(YarsException)
{
  s = "";
  Buffer b;
  b.resize(0);
  b.label = __STRING_VALUE;
  *this >> b;
  stringstream oss;
  int size = 0;
  ((Socket*)this)->__readInteger(&size, b, 0);
  for(unsigned int i = sizeof(int); i < b.size(); i++)
  {
    oss << b[i];
  }
  s = oss.str();
  return *this;
}




////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// VECTOR OF INTEGERS 
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// VECTOR OF INTEGERS - SEND
////////////////////////////////////////////////////////////////////////////////
const Socket& Socket::operator<<(const vector<int>& v) const
{
  Buffer b;
  b.label = __INTEGER_VECTOR;
  b.resize(0);
  int s = (int)v.size(); 
  ((Socket*)this)->__writeInteger(&b, s);

  for(vector<int>::const_iterator i = v.begin(); i != v.end(); i++)
  {
    int value = *i;
    ((Socket*)this)->__writeInteger(&b, value);
  }
  *this << b;
  return *this;
}


////////////////////////////////////////////////////////////////////////////////
// VECTOR OF INTEGERS - RECEIVE
////////////////////////////////////////////////////////////////////////////////
const Socket& Socket::operator>>(vector<int>& v) const throw(YarsException)
{
  v.clear();
  Buffer b;
  b.label = __INTEGER_VECTOR;
  b.resize(0);
  int vectorSize = 0;

  *this >> b;

  ((Socket*)this)->__readInteger(&vectorSize, b, 0);

  for(int i = 0; i < vectorSize; i++)
  {
    int value = 0;
    ((Socket*)this)->__readInteger(&value, b, (i+1) * sizeof(int));
    v.push_back(value);
  }
  return *this;
}





////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// VECTOR OF DOUBLES 
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// VECTOR OF DOUBLES - SEND
////////////////////////////////////////////////////////////////////////////////
const Socket& Socket::operator<<(const vector<double>& v) const
{
  Buffer b;
  b.label = __DOUBLE_VECTOR;
  b.resize(0);
  int s = (int)v.size(); 

  ((Socket*)this)->__writeInteger(&b, s);

  for(vector<double>::const_iterator i = v.begin(); i != v.end(); i++)
  {
    double value = *i;
    ((Socket*)this)->__writeDouble(&b, value);
  }
  *this << b;
  return *this;
}


////////////////////////////////////////////////////////////////////////////////
// VECTOR OF DOUBLES - RECEIVE
////////////////////////////////////////////////////////////////////////////////
const Socket& Socket::operator>>(vector<double>& v) const throw(YarsException)
{
  v.clear();
  Buffer b;
  b.label = __DOUBLE_VECTOR;
  b.resize(0);

  *this >> b;

  int vectorSize = 0;

  ((Socket*)this)->__readInteger(&vectorSize, b, 0);

  for(int i = 0; i < vectorSize; i++)
  {
    double value = 0;
    ((Socket*)this)->__readDouble(&value, b, i * sizeof(double) + sizeof(int));
    v.push_back(value);
  }
  return *this;
}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Buffer access functions
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void Socket::__writeDouble(Buffer *b, double d)
{
  char *x_ptr=reinterpret_cast<char*>(&d); 
  for(unsigned int count = 0;count < sizeof(double); count++)
  {
    b->push_back(*(x_ptr+count));
  }
}

void Socket::__readDouble(double *d, Buffer b, int startIndex)
{
  *d = 0;
  char *x_ptr=reinterpret_cast<char *>(d); 
  for(int count = sizeof(double)-1; count >= 0; count--)
  {
    *(x_ptr+count)|=b[startIndex + count]; 
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void Socket::__writeInteger(Buffer *b, int i)
{
  char *x_ptr=reinterpret_cast<char*>(&i); 
  for(unsigned int count = 0; count < sizeof(int); count++)
  {
    b->push_back(*(x_ptr+count));
  }
}

void Socket::__readInteger(int *i, Buffer b, int startIndex)
{
  *i = 0;
  char *x_ptr=reinterpret_cast<char *>(i); 
  for(int count = sizeof(int)-1; count >= 0; count--)
  {
    *(x_ptr+count)|=b[startIndex + count]; 
  }
}

void Socket::__coneverToInt(char *c, int *i)
{
  *i = 0;
  char *x_ptr=reinterpret_cast<char *>(i); 
  for(int count = sizeof(int)-1; count >= 0; count--)
  {
    *(x_ptr+count)|=c[count]; 
  }
}

void Socket::__check(const char a, const char b) throw(YarsException)
{
  if(a == b) return; // everything ok
  stringstream oss;
  oss << "Socket communication error. Awaited ";
  switch(a)
  {
    case __DOUBLE_VALUE:
      oss << "<double value>";
      break;
    case __INTEGER_VALUE:
      oss << "<integer value>";
      break;
    case __STRING_VALUE:
      oss << "<string value>";
      break;
    case __DOUBLE_VECTOR:
      oss << "<double vector>";
      break;
    case __INTEGER_VECTOR:
      oss << "<integer vector>";
      break;
    default:
      oss << "<unknown \"" << (int)b << "\">";
      break;
  }
  oss << " but received ";
  switch(b)
  {
    case __DOUBLE_VALUE:
      oss << "<double value>";
      break;
    case __INTEGER_VALUE:
      oss << "<integer value>";
      break;
    case __STRING_VALUE:
      oss << "<string value>";
      break;
    case __DOUBLE_VECTOR:
      oss << "<double vector>";
      break;
    case __INTEGER_VECTOR:
      oss << "<integer vector>";
      break;
    default:
      oss << "<unknown \"" << b << "\">";
      break;
  }
  close();
  throw YarsException(oss.str());
}



void Socket::close()
{
  if(_sock != -1)
  {
    ::shutdown(_sock, SHUT_RDWR);
    ::close(_sock);
  }
  if(_mysock != -1)
  {
    ::shutdown(_mysock, SHUT_RDWR);
    ::close(_mysock);
  }
  _sock   = -1;
  _mysock = -1;
}
