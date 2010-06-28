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
 *   Revision 1.13  2010-06-28 14:47:44  martius
 *   internal collisions are now switched on again
 *   joints do not ignore collision of connected pairs here
 *   frictionGround effects substances->works again
 *
 *   Revision 1.12  2010/01/26 09:53:06  martius
 *   changed to new collision model
 *
 *   Revision 1.11  2007/11/07 13:21:16  martius
 *   doInternal stuff changed signature
 *
 *   Revision 1.10  2006/08/11 15:44:53  martius
 *   *** empty log message ***
 *
 *   Revision 1.9  2006/07/20 17:19:44  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.8  2006/07/14 15:13:46  fhesse
 *   minor changes
 *
 *   Revision 1.6.4.9  2006/06/25 16:57:13  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.6.4.8  2006/03/30 12:34:56  martius
 *   documentation updated
 *
 *   Revision 1.6.4.7  2006/03/30 08:43:05  fhesse
 *   getTracePrimitive removed
 *
 *   Revision 1.6.4.6  2006/03/28 14:20:28  fhesse
 *   getTracePrimitive() added
 *
 *   Revision 1.6.4.5  2005/12/30 22:54:38  martius
 *   removed parentspace!
 *
 *   Revision 1.6.4.4  2005/12/21 17:35:09  martius
 *   moved to osg
 *
 *   Revision 1.6.4.3  2005/11/16 11:26:52  martius
 *   moved to selforg
 *
 *   Revision 1.6.4.2  2005/11/15 12:29:26  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.6.4.1  2005/11/14 17:37:17  martius
 *   moved to selforg
 *
 *   Revision 1.6  2005/11/09 13:26:21  martius
 *   added factorSensor
 *
 *   Revision 1.5  2005/10/06 17:14:24  martius
 *   switched to stl lists
 *
 *   Revision 1.4  2005/09/22 12:24:37  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.3  2005/08/31 17:18:15  fhesse
 *   setTextures added, Mass is now sphere (not box anymore)
 *
 *   Revision 1.2  2005/07/26 17:04:21  martius
 *   lives in its own space now
 *
 *   Revision 1.1  2005/07/21 12:17:04  fhesse
 *   new hurling snake, to do add collision space, clean up, comment
 *
 *         
 *                                                                 *
 ***************************************************************************/
#ifndef __HURLINGSNAKE_H
#define __HURLINGSNAKE_H

#include "oderobot.h"
#include <selforg/configurable.h>
#include "primitive.h"
#include "joint.h"

namespace lpzrobots {

  /**
   * Hurling snake is a string a beats.
   * 
   */
  class HurlingSnake : public OdeRobot{
  public:
    /**
     * Constructor
     */
    HurlingSnake(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const std::string& name);
  
    /// update the subcomponents
    virtual void update();

    /** sets the pose of the vehicle
	@param pose desired 4x4 pose matrix
    */
    virtual void place(const osg::Matrix& pose);

    /** this function is called in each timestep. It should perform robot-internal checks, 
	like space-internal collision detection, sensor resets/update etc.
	@param globalData structure that contains global data from the simulation environment
    */
    //    virtual void doInternalStuff(GlobalData& globalData);
  

    /** returns actual sensorvalues
	@param sensors sensors scaled to [-1,1] 
	@param sensornumber length of the sensor array
	@return number of actually written sensors
    */
    virtual int getSensors(sensor* sensors, int sensornumber);

    /** sets actual motorcommands
	@param motors motors scaled to [-1,1] 
	@param motornumber length of the motor array
    */
    virtual void setMotors(const motor* motors, int motornumber);

    /** returns number of sensors
     */
    virtual int getSensorNumber();

    /** returns number of motors
     */
    virtual int getMotorNumber();

    /** returns a vector with the positions of all segments of the robot
	@param poslist vector of positions (of all robot segments) 
	@return length of the list
    */
    virtual int getSegmentsPosition(std::vector<Position> &poslist);
  
    /** The list of all parameters with there value as allocated lists.       
    */
    virtual paramlist getParamList() const;
  
    virtual paramval getParam(const paramkey& key) const;

    virtual bool setParam(const paramkey& key, paramval val);

  protected:
    /** the main object of the robot, which is used for position and speed tracking */
    virtual Primitive* getMainPrimitive() const { return object[(NUM-1)/2] /*(center)*/; }
    //virtual Primitive* getMainPrimitive() const { return object[NUM-1] /*(head element)*/; }


  private:

    /** creates vehicle at desired pose
	@param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose); 

    /** destroys robot and space
     */
    virtual void destroy();
       
    bool created;      // true if robot was created


    Position initial_pos;    // initial position of robot

    int NUM;	   /* number of beats */
    double MASS;	   /* mass of a beats */
    double RADIUS;   /* sphere radius */

    Joint* joint[9];
    Primitive* object[10];

    Pos oldp;

    int sensorno;
    int motorno;

    paramval factorForce;
    paramval frictionGround;
    paramval factorSensor;

  };

}

#endif
 
