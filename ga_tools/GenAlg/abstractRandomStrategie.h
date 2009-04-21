/*
 * abstractRandomStrategie.h
 *
 *  Created on: 21.04.2009
 *      Author: robot12
 */

#ifndef ABSTRACTRANDOMSTRATEGIE_H_
#define ABSTRACTRANDOMSTRATEGIE_H_

#include "types.h"

virtual class abstractRandomStrategie {
public:
	abstractRandomStrategie();
	virtual ~abstractRandomStrategie();

	virtual UNKNOWN_DATA_TYP_PTR getRandomValue(void) = 0;
};

#endif /* ABSTRACTRANDOMSTRATEGIE_H_ */
