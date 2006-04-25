#/***************************************************************************
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
 ***************************************************************************/


#include <math.h>
#include <ode/ode.h>
#include <iostream>
#include <typeinfo>

#include <vector>
#include <list>


#include "oderobot.h"
#include "joint.h"



#ifndef component_h
#define component_h


namespace lpzrobots
{

/**
 * Component
 *
 *
 */
class Component : public OdeRobot
{
    typedef struct
    {
	Component* subcomponent;
	Joint* joint;
    } componentConnection;

    typedef struct
    {
	bool completesensormode; //if true, the controler of the Component also gets the sensor values of the robot of the component
	bool completemotormode; //if true, the controler of the component also controls the robot of the component
	double max_force;
	double speed;
    } ComponentConf;

 private:
    ComponentConf conf;
    
    //only one of this two should reference to an object
    Primitive* simplePrimitive;
    OdeRobot* robot;

    vector <componentConnection> connection;

 public:

    Component ( const OdeHandle &odeHandle, const OsgHandle &osgHandle, const ComponentConf& conf);
    
    ~Component ();

static ComponentConf getDefaultConf()
{
    ComponentConf conf;
    conf.completesensormode = true;
    conf.completemotormode = false;
    conf.max_force = 8;
    conf.speed = 1;
    return conf;
}

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
 *Sets the reference to the Primitive , which belongs to the component, but only if there is no robot set, belonging to the component.
 *@return true if the reference could be set; false else
 **/
virtual bool setSimplePrimitive ( Primitive* newprimitive );

/**
 *Sets the reference to the robot for the component, but only if there is no Primitive set belonging to the component.
 *Overwriting an existing robot reference is possible, also to set it NULL, and then set a reference to a Primitive with setSimplePrimitive ( .. ).
 *return true if the reference could be set; false else
 **/
virtual bool setRobot ( OdeRobot* newrobot );

virtual OdeRobot* getRobot (); //returns a reference to the robot belonging to the component, if there is no robot it is an NULL pointer, then try getMeinPrimitive, because there is only a solid Primitive not a complex robot for this component

virtual Primitive* getMainPrimitive () const;//overload this in the robot implementation.; should be the main-Primitive from the first componentConnection in the vector

/**
 *Gets the Number of subcomponents of this component.
 *@return Number of subcomponents
 **/
virtual int getNumberSubcomponents ();

    
/**
 *Gets the Number of all Subcomponents recursivly connected.
 *@return Number of subcomponents
 **/
virtual int getNumberSubcomponentsAll ();

/**
 *This method adds an existing Component as a subcomponent to this component
 *@param subcomponent to add
 *@param reference to external created joint, which connects both components
 **/
virtual void addSubcomponent ( Component* newsubcomponent , Joint* newconnectingjoint );

/**
 *This method removes an existing Component as a subcomponent of this component. This also removes all Subcomponents of the subcomponent.
 *@param subcomponent number to remove
 *@return reference to the removed subcomponent, so that it could be used to do other things
 **/
virtual Component* removeSubcomponent ( int removedsubcomponentnumber );

/**
 *This method removes an existing Component as a subcomponent of this component. This also removes all Subcomponents of the subcomponent.
 *@param subcomponent to remove
 *@return reference to the removed subcomponent, so that it could be used to do other things
 **/
virtual Component* removeSubcomponent ( Component* removedsubcomponent );
     
};


}
#endif
