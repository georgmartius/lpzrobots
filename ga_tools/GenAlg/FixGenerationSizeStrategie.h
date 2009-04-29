/*
 * FixGenerationSizeStrategie.h
 *
 *  Created on: 29.04.2009
 *      Author: robot12
 */

#ifndef FIXGENERATIONSIZESTRATEGIE_H_
#define FIXGENERATIONSIZESTRATEGIE_H_

#include "types.h"

#include "IGenerationSizeStrategie.h"

class FixGenerationSizeStrategie: public IGenerationSizeStrategie {
public:
	FixGenerationSizeStrategie();
	virtual ~FixGenerationSizeStrategie();

	virtual int calcGenerationSize(/*parameters unknown*/);
};

#endif /* FIXGENERATIONSIZESTRATEGIE_H_ */
