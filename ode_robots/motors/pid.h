/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 *   $Log$
 *   Revision 1.6  2005-11-09 14:08:48  martius
 *   *** empty log message ***
 *
 *   Revision 1.5  2005/11/09 13:28:24  fhesse
 *   GPL added
 *                                                                * 
***************************************************************************/


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
