/*
 * types.h
 *
 *  Created on: 03.12.2008
 *      Author: wolfgang
 */

#ifndef TYPES_H_
#define TYPES_H_
#include <QtGui>

typedef unsigned char QByte;
typedef unsigned short QWord;
typedef QByte uint8_t;

struct MessageIsp_t{
  uint8_t startDelemiter;
  uint8_t lengthHigh;
  uint8_t lengthLow;
  uint8_t apiIdentifier;
  // ----------------------
  uint8_t msgGroup;
  uint8_t msgCode;
  uint8_t data[];
};


#endif /* TYPES_H_ */
