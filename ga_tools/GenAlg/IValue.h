/*
 * IValue.h
 *
 *  Created on: 22.04.2009
 *      Author: robot12
 */

#ifndef IVALUE_H_
#define IVALUE_H_

class IValue {
public:
	IValue();
	virtual ~IValue();

	IValue* operator*(IValue& value) = 0;
};

#endif /* IVALUE_H_ */
