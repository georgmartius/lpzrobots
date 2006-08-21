/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    marcel@informatik.uni-leipzig.de                                     *
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
 ***************************************************************************/


#include "component.h"

#ifndef robotcomponent_h
#define robotcomponent_h


namespace lpzrobots
{
/**
 * This is a special class of the component system. The physical object that belongs to objects of this class is a robot whose class is derived from OdeRobot.
 * The sensor an motor values these component sends and gets could be used to controll the robot that belongs to this component.
 * If this controling method should be used there are two controling parameters that tell the component, how to handle its sensor/motor values.
 * These are the params completemotormode and complete sensormode of the componentConf. These two parameters control if the sensor/motor values will eb send to the robot of the component or not. If thei are not sent to the robot the robot needs its own controller.
 */
class RobotComponent : public Component
{

 private:
    
    OdeRobot* robot;

 public:

    RobotComponent ( const OdeHandle &odeHandle, const OsgHandle &osgHandle, const ComponentConf& conf);
    
    ~RobotComponent ();

 public:

/**
 *Use this, to get all sensor values of all the joints of all subcomponents, and the sensors of all robots, belonging to all subcompionents.
 *The sensor values have the following sequence:
 *values of the component connecting joints, values of the robot of the component,
 *values of component connecting joints of the first subcomponent, values of the robot of the first subcomponent, ...
 *@robot sensor values of the connecting joints of this component and all subcomponents
 **/
virtual int 	getSensors (sensor *sensors, int sensornumber); //returns actual sensorvalues; only for the connecting joints

/**
 *Sets the motor values for the joints connecting the component with its subcomponents, an recursivly the joints of all subComponents.
 *The motors of all robots of the subcomponents is not set.
 *@param 
 **/
virtual void 	setMotors (const motor *motors, int motornumber); //sets actual motorcommands; only for the connecting joints

virtual int 	getSensorNumber (); //returns number of sensors; recursivly adding of the number of sensors all subcomponents and the robots of all Subcomponents.

virtual int 	getMotorNumber (); //returns number of motors; recursivly adding of the number of sensors all subcomponents; at the moment only counts Hinge-, Slider-, Hinge2 and Universal-Joints; The Motor-Numbers of the robots of the Components is not counted.

virtual void 	update ();//update the OSG notes here; update of the underlying robot or Primitive and recursive update of all Components in the Connection-vector

virtual void 	place (const Pos &pos);//sets the vehicle to position pos - desired position of the robot; the first component is seen as center of the robot, on which the position pos refers; also recursive place of all subComponents
virtual void 	place (const osg::Matrix &pose);//sets the pose of the vehicle; also recursive place of all subComponents; does nothing at the moment

virtual bool 	collisionCallback (void *data, dGeomID o1, dGeomID o2);// checks for internal collisions and treats them.; should do nothing, because there should not be any ode-objects belonging to the component, which are not handled elsewhere....and what is with Primitives? are they automaticaly handled?

virtual void 	doInternalStuff (const GlobalData &globalData);// this function is called in each timestep.; maybee usefull

// virtual void 	setColor (const Color &col); 	sets color of the robot; not nessecary

virtual Position getPosition () const; //returns position of the object; relates to the robot or Primitive belonging to the component

//virtual Position getSpeed () const;//returns linear speed vector of the object; must be computed from all sub-robots

//virtual matrix::Matrix 	getOrientation () const;//returns the orientation of the object;


/**
 *This is only a simple function, calculating the coordinates of the point exactly between two directly connected components.
 *@return Vector containing the Position
 *@param index number of the position
 **/
virtual osg::Vec3 getPositionbetweenComponents ( Component* component );

/**
 *Sets the reference to the robot for the component, but only if there is no Primitive assigned to the component.
 *Overwriting an existing robot reference is possible, also to set it NULL, and then set a reference to a Primitive with setSimplePrimitive ( .. ).
 *But first the robot reference should be saved elsewhere or it won't be updated graficaly
 *return true if the reference could be set; false else
 **/
virtual bool setRobot ( OdeRobot* newrobot );

/**
 *returns a reference to the robot belonging to the component, if there is no robot it is an NULL pointer, then try getMeinPrimitive, because there is only a solid Primitive not a complex robot for this component
 *@return the robot asigned to the component
 **/
virtual OdeRobot* getRobot (); //

/**
 *return reference to the simple Primitive, or to the main Primitive of the robot assigend to the component. If nothimng is assigned, NULL is returned.
 **/
virtual Primitive* getMainPrimitive () const;
     
};


}
#endif
