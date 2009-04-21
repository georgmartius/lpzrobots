/*
 * abstractMutationFactorOptimizerStrategie.h
 *
 *  Created on: 21.04.2009
 *      Author: robot12
 */

#ifndef ABSTRACTMUTATIONFACTOROPTIMIZERSTRATEGIE_H_
#define ABSTRACTMUTATIONFACTOROPTIMIZERSTRATEGIE_H_

#include "types.h"

#include <vector>

virtual class abstractMutationFactorOptimizerStrategie {
public:
	abstractMutationFactorOptimizerStrategie();
	virtual ~abstractMutationFactorOptimizerStrategie();

	virtual UNKNOWN_DATA_TYP_PTR updateMutationFactor(std::vector<Gen*>& gene) = 0;
};

#endif /* ABSTRACTMUTATIONFACTOROPTIMIZERSTRATEGIE_H_ */
