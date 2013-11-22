/***************************************************************************
 *   Copyright (C) 2010 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   ANY COMMERCIAL USE FORBIDDEN!                                         *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                            *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/
#ifndef __RINGBUFFER_H
#define __RINGBUFFER_H

#include <vector>
#include <assert.h>

template <typename T>
class RingBuffer {
public:
  RingBuffer()
    :buffersize(0)
  { }

  RingBuffer(int size)
  : buffersize(size) {
    buffer.resize(size);
  }

  /// sets size of buffer and initializes buffer elements.
  void init(int size, const T& t){
    buffersize = size;
    buffer.resize(size, t);
  }

  int getBufferSize() const {
    return buffersize;
  }

  /** returns object at index.
      Index can be larger than buffersize, it will be wrapped. Negative index means 0.
  */
  T& get(int index){
    assert(buffersize>0);
    if(index < 0) index=0;
    return buffer[index%buffersize];
  }

  const T& get(int index) const {
    assert(buffersize>0);
    if(index < 0) index=0;
    return buffer[index%buffersize];
  }

  /// see get()
  T& operator[] (int index){
    return get(index);
  }
  /// see get()
  const T& operator[] (int index) const {
    return get(index);
  }

protected:
  std::vector<T> buffer;
  int buffersize;
};

#endif
