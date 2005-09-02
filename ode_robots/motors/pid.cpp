/************************************************************************/
/*pid.cpp							*/
/************************************************************************/

#include "pid.h"

PID::PID ( double start_KP = 500 , double start_KI = 0 , double start_KD = 20 )
{
	KP = start_KP;
	KI = start_KI;
	KD = start_KD;

	P=D=I=0;

	targetposition = 0;
	
	position = 0;
	lastposition = 0;
	error = 0;
	alpha = 0.98;
}

void PID::setTargetPosition ( double newpos )
{
	targetposition = newpos;
}

double PID::getTargetPosition ( )
{
	return targetposition;
}

double PID::step ( double newsensorval )
{
	last2position = lastposition;
	lastposition = position;
	position = newsensorval;
	
	return stepWithD(newsensorval, lastposition - position);
}

double PID::stepWithD ( double newsensorval, double derivative ){
	position = newsensorval;

	lasterror = error;
	error = targetposition - position;
	
	P = error;
	I += (1-alpha) * (error * KI - I);
	D = -derivative * KD;
	// D = -( 3*position - 4 * lastposition + last2position ) * KD;
	//double maxforce = KP/10;
	//P = P > maxforce ? maxforce : (force < -maxforce ? -maxforce : P);

	force = KP*(P + I + D);
	return force;

}

