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
 ***************************************************************************/


#include "component.h"

#ifndef simplecomponent_h
#define simplecomponent_h


namespace lpzrobots
{
/**
 * SimpleComponent
 *
 *
 */
class SimpleComponent : public Component
{

 private:
    //only one of this two should reference to an object
    Primitive* simplePrimitive;

 public:

    SimpleComponent ( const OdeHandle &odeHandle, const OsgHandle &osgHandle, const ComponentConf& conf);
    
    ~SimpleComponent ();

 public:

/**
 *Use this, to get all sensor values of all the joints of all subcomponents, and the sensors of all robots, belonging to all subcomponents.
 *The sensor values have the following sequence:
 *values of the component connecting joints, values of the robot of the component,
 *values of component connecting joints of the first subcomponent, values of the robot of the first subcomponent, ...
 *@param robot sensor values of the connecting joints of this component and all subcomponents
 **/
virtual int 	getSensors (sensor *sensors, int sensornumber); //returns actual sensorvalues; only for the connecting joints

/**
 *Sets the motor values for the joints connecting the component with its subcomponents, an recursivly the joints of all subComponents.
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
 *Sets the reference to the Primitive for the component, but only if there is no robot assigned to the component.
 *Overwriting an existing Primitive reference is possible, also to set it NULL, and then set a reference to a Primitive with setSimplePrimitive ( .. ).
 *But first the Primitive reference should be saved elsewhere or it won't be updated graficaly
 *@return true if the reference could be set; false else
 **/
virtual bool setSimplePrimitive ( Primitive* newprimitive );

/**
 *return reference to the simple Primitive, or to the main Primitive of the robot assigend to the component. If nothimng is assigned, NULL is returned.
 **/
virtual Primitive* getMainPrimitive () const;

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
