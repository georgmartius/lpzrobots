/*
 * IValue.h
 *
 *  Created on: 22.04.2009
 *      Author: robot12
 */

#ifndef IVALUE_H_
#define IVALUE_H_

#include "types.h"

#include "inspectable.h"

class IValue : public Inspectable{
public:
	IValue();
	virtual ~IValue();

	IValue* operator*(IValue& value)const = 0;
};

#endif /* IVALUE_H_ */
