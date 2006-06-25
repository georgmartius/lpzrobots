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
 *   Revision 1.4.4.4  2006-06-25 16:57:13  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.4.4.3  2006/01/10 22:25:09  martius
 *   moved to osg
 *
 *   Revision 1.1.2.1  2006/01/10 17:15:04  martius
 *   was sphererobotarms
 *   moved to osg
 *
 *   Revision 1.18.4.2  2005/11/15 12:29:27  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.18.4.1  2005/11/14 17:37:18  martius
 *   moved to selforg
 *
 *   Revision 1.18  2005/11/09 13:26:57  martius
 *   irsensorrange
 *
 *   Revision 1.17  2005/11/09 13:24:42  martius
 *   added GPL
 *
 ***************************************************************************/

#include <assert.h>
#include <matrix.h>
#include <osg/Matrix>
#include "primitive.h"
#include "forcedsphere.h"

using namespace osg;

namespace lpzrobots {

  /**
   *constructor
   **/
  ForcedSphere::ForcedSphere ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		 const char* name, double radius, double max_force)
    : OdeRobot ( odeHandle, osgHandle, name, "$ID$" ), radius(radius), max_force(max_force)
  {

    created = false;
    object[0] = 0;
    
  }
	
  ForcedSphere::~ForcedSphere()
  {
    destroy(); 
  }

  void ForcedSphere::update()
  {
    if(object[0]) object[0]->update();
  }
  
  /**
   *Writes the sensor values to an array in the memory.
   *@param sensor* pointer to the array
   *@param sensornumber length of the sensor array
   *@return number of actually written sensors
   **/
  int ForcedSphere::getSensors ( sensor* sensors, int sensornumber )
  {  
    int len=0;
    matrix::Matrix A = odeRto3x3RotationMatrix ( dBodyGetRotation ( object[0]->getBody() ) );

    // z-coordinate of axis position in world coordinates
    len += A.row(2).convertToBuffer(sensors+len, sensornumber-len);  
    // rotation matrix - 9 (vectors of all axis in world coordinates
    //len += A.convertToBuffer(sensors + len , sensornumber -len);
      
    return len;
  }

  /**
   *Reads the actual motor commands from an array, an sets all motors of the snake to this values.
   *It is an linear allocation.
   *@param motors pointer to the array, motor values are scaled to [-1,1] 
   *@param motornumber length of the motor array
   **/
  void ForcedSphere::setMotors ( const motor* motors, int motornumber ) {
    if (motornumber==2){
      dBodyAddForce(object[0]->getBody(), motors[0]*max_force, motors[1]*max_force, 0);
    }
  }


  void ForcedSphere::place(const osg::Matrix& pose){
    osg::Matrix p2;
    p2 = pose * osg::Matrix::translate(osg::Vec3(0, 0, radius)); 
    create(p2);    
  };


  void ForcedSphere::doInternalStuff(const GlobalData& global){
  }

  /**
   *This is the collision handling function for sphere robots.
   *This overwrides the function collisionCallback of the class robot.
   *@param data
   *@param o1 first geometrical object, which has taken part in the collision
   *@param o2 second geometrical object, which has taken part in the collision
   *@return true if the collision was threated  by the robot, false if not
   **/
  bool ForcedSphere::collisionCallback(void *data, dGeomID o1, dGeomID o2) {
    return false; // let the standard routine do it for us
  }


  /**
   *Returns the number of motors used by the snake.
   *@return number of motors
   **/
  int ForcedSphere::getMotorNumber(){
    return 2;
  }

  /**
   *Returns the number of sensors used by the robot.
   *@return number of sensors
   **/
  int ForcedSphere::getSensorNumber() {
    return 3;
  }


  /** creates vehicle at desired position 
      @param pos struct Position with desired position
  */
  void ForcedSphere::create(const osg::Matrix& pose){
    if (created) {
      destroy();
    }

    object[0] = new Sphere(radius);
    object[0]->init(odeHandle, radius, osgHandle);
    object[0]->setPose(pose);    
  }


  /** destroys vehicle and space
   */
  void ForcedSphere::destroy(){
    if (created){
      for (int i=0; i<1; i++){
	if(object[i]) delete object[i];
      }
    }
    created=false;
  }

}

