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
 *   Revision 1.7  2006-07-14 12:23:32  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.6.4.3  2006/02/07 15:51:56  martius
 *   axis, setpower
 *
 *   Revision 1.6.4.2  2006/01/10 14:48:59  martius
 *   indentation
 *   #ifdef clausel
 *
 *   Revision 1.6.4.1  2005/12/20 17:53:42  martius
 *   changed to Joints from joint.h
 *   new servos for universal and hinge2
 *
 *   Revision 1.6  2005/11/09 14:08:48  martius
 *   *** empty log message ***
 *
 *   Revision 1.5  2005/11/09 13:28:24  fhesse
 *   GPL added
 *                                                                * 
***************************************************************************/
#ifndef __PID_H
#define __PID_H

namespace lpzrobots {

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
  /// KP is used as a general koefficient. KI and KD can be tuned without dependence of KP
  PID ( double KP = 100 , double KI = 2.0 , double KD = 0.3 );
 
  void setKP(double KP);
 
  void setTargetPosition ( double newpos );
		
  double getTargetPosition ();
		
  double step ( double newsensorval );
  double stepWithD ( double newsensorval, double derivative );
};

}

#endif
