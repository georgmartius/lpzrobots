/************************************************************************/
/*pid.h																	*/
/************************************************************************/ 

#include <drawstuff/drawstuff.h>
#include <ode/ode.h>
#include <vector>

class PID
{
  //*********************attributes***************
  //private:
public:
		
  double position;
  double lastposition;
  double last2position;
		
  double error;
  double lasterror;

  double targetposition;
		
  double KP;
  double KD;
  double KI;
  double alpha;
	       	
  double P;
  double D;
  double I;
	
  double force;

  //*********************methods******************
public :
  PID ( double start_KP , double start_KI , double start_KD );

  void setTargetPosition ( double newpos );
		
  double getTargetPosition ();
		
  double step ( double newsensorval );
  double stepWithD ( double newsensorval, double derivative );
};
