/*
 * main.cpp
 *
 *  Created on: 04.05.2009
 *      Author: robot12
 */

#include "SingletonGenAlgAPI.h"
#include <selforg/randomgenerator.h>

void main() {
	RandGen random;

	IRandomStrategy* randomSt = SingletonGenAlgAPI::getInstance()->createDoubleRandomStrategy(&random);

	IValue* mutFa = SingletonGenAlgAPI::getInstance()->createDoubleValue(0.3);
	IMutationFactorStrategy* mutFaSt = SingletonGenAlgAPI::getInstance()->createFixMutationFactorStrategy(mutFa);

	IMutationStrategy* mutSt = SingletonGenAlgAPI::getInstance()->createValueMutationStrategy()
}
