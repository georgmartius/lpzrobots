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
 *  A robot with 4 wheels based on nimm4 with IR sensors                   *
 *                                                                         *
 *   $Log$
 *   Revision 1.4  2009-05-11 17:03:07  martius
 *   minor substance change
 *
 *   Revision 1.3  2008/04/23 07:17:16  martius
 *   makefiles cleaned
 *   new also true realtime factor displayed,
 *    warning if out of sync
 *   drawinterval in full speed is 10 frames, independent of the speed
 *
 *   Revision 1.2  2007/11/07 13:21:15  martius
 *   doInternal stuff changed signature
 *
 *   Revision 1.1  2007/08/24 11:49:35  martius
 *   initial
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __FOUR_WHEELED__
#define __FOUR_WHEELED__

#include "nimm4.h"
#include "raysensorbank.h"

namespace lpzrobots {

  class Primitive; 
  class Hinge2Joint; 

  typedef struct {
    double size;
    double force;
    double speed;
    bool sphereWheels;
    bool irFront;
    bool irBack;
    bool irSide;
    double irRangeFront;
    double irRangeBack;
    double irRangeSide;
    Substance wheelSubstance;
  } FourWheeledConf;
  
  /** Robot is based on nimm4 with 
      4 wheels and a capsule like body   
  */
  class FourWheeled : public Nimm4{
  public:
  
    /**
     * constructor of nimm4 robot
     * @param odeHandle data structure for accessing ODE
     * @param osgHandle ata structure for accessing OSG
     * @param conf configuration structure
     * @param name name of the robot
     */
    FourWheeled(const OdeHandle& odeHandle, const OsgHandle& osgHandle, FourWheeledConf conf, const std::string& name);

    static FourWheeledConf getDefaultConf(){
      FourWheeledConf conf;
      conf.size=1;
      conf.force=3;
      conf.speed=15;
      conf.sphereWheels=true;
      conf.irFront=false;
      conf.irBack=false;
      conf.irSide=false;
      conf.irRangeFront=3;
      conf.irRangeSide=2;
      conf.irRangeBack=2;
      conf.wheelSubstance.toRubber(40);
      return conf;
    }

    virtual ~FourWheeled();

    /**
     * updates the OSG nodes of the vehicle
     */
    virtual void update();

    virtual int getSensorNumber();

    /** returns actual sensorvalues
	@param sensors sensors scaled to [-1,1] 
	@param sensornumber length of the sensor array
	@return number of actually written sensors
    */
    virtual int getSensors(sensor* sensors, int sensornumber);

    /** this function is called in each timestep. It should perform robot-internal checks, 
	like space-internal collision detection, sensor resets/update etc.
	@param globalData structure that contains global data from the simulation environment
    */
    virtual void doInternalStuff(GlobalData& globalData);


  protected:
    /** creates vehicle at desired pose
	@param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose); 

    /** destroys vehicle and space
     */
    virtual void destroy();

    FourWheeledConf conf;
    RaySensorBank irSensorBank; // a collection of ir sensors
    Primitive* bumpertrans;
    Primitive* bumper;
  };

}

#endif
