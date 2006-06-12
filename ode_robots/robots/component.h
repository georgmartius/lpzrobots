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
    typedef struct
    {
	bool completesensormode; //if true, the controler of the Component also gets the sensor values of the robot of the component
	bool completemotormode; //if true, the controler of the component also controls the robot of the component
	double max_force;
	double speed;
    } ComponentConf;

/**
 * Component
 *
 *
 */
class Component : public OdeRobot
{
 public:
    ComponentConf conf;

 protected:
    typedef struct
    {
	Component* subcomponent;
	Joint* joint;
	bool softlink; //if true the connection ends the recursion, false = normal
    } componentConnection;

    vector <componentConnection*> connection;
public: 
    Component* originComponent;

 public:

    Component ( const OdeHandle &odeHandle, const OsgHandle &osgHandle, const ComponentConf& conf);
    
    ~Component ();

static ComponentConf getDefaultConf()
{
    ComponentConf conf;
    conf.completesensormode = true;
    conf.completemotormode = false;
    conf.max_force = 1;
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
virtual int 	getSensors (sensor *sensors, int sensornumber) = 0; //returns actual sensorvalues; only for the connecting joints

/**
 *Sets the motor values for the joints connecting the component with its subcomponents, an recursivly the joints of all subComponents.
 *The motors of all robots of the subcomponents is not set.
 *@param 
 **/
virtual void 	setMotors (const motor *motors, int motornumber) = 0; //sets actual motorcommands; only for the connecting joints

virtual int 	getSensorNumber () = 0; //returns number of sensors; recursivly adding of the number of sensors all subcomponents and the robots of all Subcomponents.

virtual int 	getMotorNumber () = 0; //returns number of motors; recursivly adding of the number of sensors all subcomponents; at the moment only counts Hinge-, Slider-, Hinge2 and Universal-Joints; The Motor-Numbers of the robots of the Components is not counted.

virtual void 	update () = 0;//update the OSG notes here; update of the underlying robot or Primitive and recursive update of all Components in the Connection-vector

virtual void 	place (const Pos &pos) = 0;//sets the vehicle to position pos - desired position of the robot; the first component is seen as center of the robot, on which the position pos refers; also recursive place of all subComponents

virtual bool 	collisionCallback (void *data, dGeomID o1, dGeomID o2);// checks for internal collisions and treats them.; should do nothing, because there should not be any ode-objects belonging to the component, which are not handled elsewhere....and what is with Primitives? are they automaticaly handled?

virtual void 	doInternalStuff (const GlobalData &globalData);// this function is called in each timestep.; maybee usefull

// virtual void 	setColor (const Color &col); 	sets color of the robot; not nessecary

virtual Position getPosition () const = 0; //returns position of the object; relates to the robot or Primitive belonging to the component

//virtual Position getSpeed () const;//returns linear speed vector of the object; must be computed from all sub-robots

//virtual matrix::Matrix 	getOrientation () const;//returns the orientation of the object;


/**
 *This is only a simple function, calculating the coordinates of the point exactly between two directly connected components.
 *@return Vector containing the Position
 *@param index number of the position
 **/
virtual osg::Vec3 getPositionbetweenComponents ( Component* component );

/**
 *return reference to the simple Primitive, or to the main Primitive of the robot assigend to the component. If nothimng is assigned, NULL is returned.
 **/
virtual Primitive* getMainPrimitive () const = 0;

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

/**
 *This method looks if a special component is a subcomponent of this component. Only direct subcomponents are registrated.
 *@param the component, which could be a subcomponent of this component
 *@return true if it is a subcomponent, false if not
 **/
virtual bool hasSubcomponent ( Component* subcomp );

/**
 *This method looks if a special component is a subcomponent of this component and all its recursive subcomponents.
 *@param the component, which could be a subcomponent
 *@return true if it is a subcomponent, false if not
 **/
virtual bool hasSubcomponentAll ( Component* subcomp );

/**
 *This method looks if a special component somehow connected to this component.
 *@param the component, which could be connected
 *@return true if it is connected, false if not
 **/
virtual bool isComponentConnected ( Component* connectedComp );

/**
 *This garants an direct access to the connections between the components. Be carefull with using the given references. 
 *@param the number of the connection
 *@return the reference of connection element
 **/
virtual componentConnection* getConnection ( int connectionnumber );

/**
 *Sets the connection between the component and one of its subcomponents to be a softlink. That means that the recusion for this branch stops here.
 *@param number of the subcomponent in the subcomponent list of the component
 *@param true = connection becomes a softlink
 **/
virtual bool setSoftlink ( unsigned int position , bool state );
     
};


}
#endif
