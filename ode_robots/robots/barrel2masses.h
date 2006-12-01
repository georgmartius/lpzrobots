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
 *                                                                 *
 ***************************************************************************
 *                                                                         *
 * cylinder like Robot inspired by Julius Popp's Adam.                     *
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2006-12-01 16:21:15  martius
 *   like sphere3masses, but with 2 masses and cylindric body
 *
 *
 *                                                                 *
 ***************************************************************************/

#ifndef __BARREL2MASSES_H
#define __BARREL2MASSES_H

#include "primitive.h"
#include "joint.h"
#include "sliderservo.h"
#include "oderobot.h"
#include "raysensorbank.h"
#include "sphererobot3masses.h"

namespace lpzrobots {

/*
  parameters for nice rolling modes:

    Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();  
    conf.pendularrange  = 0.3; 
    conf.motorsensor=false;
    conf.axisZsensor=true;
    conf.axisXYZsensor=false;
    conf.spheremass   = 1;
    sphere1 = new Barrel2Masses ( odeHandle, osgHandle.changeColor(Color(0.0,0.0,1.0)), 
				  conf, "Barrel1", 0.2); 
    sphere1->place ( osg::Matrix::rotate(M_PI/2, 1,0,0));

    controller = new InvertMotorNStep();    
    controller->setParam("steps", 2);    
    controller->setParam("adaptrate", 0.0);    
    controller->setParam("epsC", 0.03);    
    controller->setParam("epsA", 0.05);    
    controller->setParam("rootE", 3);    
    controller->setParam("logaE", 0);    
    
    One2OneWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );

*/

/**
   A barrel-like robot with 2 internal masses, which can slide on their orthogonal axes.
   It is the small brother of the Sphererobot3Masses.
   This robot was inspired by Julius Popp (http://sphericalrobots.com)
*/
class Barrel2Masses : public Sphererobot3Masses
{
public:

  /**
   * Constructor. It is configured with the configuration object of Sphererobot3Masses. 
   Just two of the 3 axis are used. The worldZaxissensor  and irAxis3 has no meaning here.
   **/ 
  Barrel2Masses ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		       const Sphererobot3MassesConf& conf, const std::string& name, double transparency=0.5 );
  
  virtual ~Barrel2Masses();
	
  /// default configuration
  static Sphererobot3MassesConf getDefaultConf(){
    Sphererobot3MassesConf c;
    c.diameter     = 1;
    c.spheremass   = .3;// 0.1
    c.pendularmass  = 1.0;
    c.pendularrange  = 0.25; // range of the slider from center in multiple of diameter [-range,range]
    c.axisZsensor = true;
    c.axisXYZsensor = false;  
    c.worldZaxissensor = false; // this has no sence here!
    c.motorsensor = false;  
    c.irAxis1=false;
    c.irAxis2=false;
    c.irAxis3=false;
    c.drawIRs=true;
    c.irsensorscale=1.5;
    c.irCharacter=1;  
    return c;
  }
	
  virtual int getSensors ( sensor* sensors, int sensornumber );
  virtual int getSensorNumber();
	
protected:

  /// The cylinder (main body) lies on the ground, that it is rotating about the z-axis
  virtual void create(const osg::Matrix& pose); 

};

}

#endif
