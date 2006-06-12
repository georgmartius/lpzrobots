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

#ifndef atomcomponent_h
#define atomcomponent_h


namespace lpzrobots
{

    typedef struct
    {
	double core_radius;
	double shell_radius;
	double mass;
	int max_bindings; //maximum possible number of fused atoms on this atom
	double binding_energy; //the value of kinetic energy that is necessary, to fuse two AtomComponents

    } AtomConf;


/**
 * AtomComponent
 *
 *
 */
class AtomComponent : public Component
{

 public:
    AtomConf atomconf;

 protected:
/*    typedef struct
    {
	Component* subcomponent;
	Joint* joint;
	bool softlink; //if true the connection ends the recursion, false = normal
	double binding_strength;
    } componentConnection;
*/
 private:
    //only one of this two should reference to an object
    OSGSphere* core;
    Primitive* shell;


 public:

    AtomComponent ( const OdeHandle &odeHandle, const OsgHandle &osgHandle, const ComponentConf& conf, const AtomConf& aconf );
    
    ~AtomComponent ();


static AtomConf getDefaultAtomConf()
{
    AtomConf conf;

    conf.core_radius = 0.05;
    conf.shell_radius = 0.1;
    conf.mass = 1;
    conf.max_bindings = 4;
    conf.binding_energy = 3;

    return conf;
}


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

virtual bool    shellCollision ( dGeomID o1 , dGeomID o2 ); //testing function for collisions with the atom shell

virtual void 	doInternalStuff (const GlobalData &globalData);// this function is called in each timestep.; maybee usefull

// virtual void 	setColor (const Color &col); 	sets color of the robot; not nessecary

virtual Position getPosition () const; //returns position of the object; relates to the robot or Primitive belonging to the component

//virtual Position getSpeed () const;//returns linear speed vector of the object; must be computed from all sub-robots

/**
 *This is only a simple function, calculating the coordinates of the point exactly between two directly connected components.
 *@return Vector containing the Position
 *@param index number of the position
 **/
virtual osg::Vec3 getPositionbetweenComponents ( Component* component );

/**
 *@return reference to the simple Primitive, or to the main Primitive of the robot assigend to the component. If nothimng is assigned, NULL is returned.
 **/
virtual Primitive* getMainPrimitive () const;

/**
 *Calculates the linear force the Component has at the time of calling this function
 *@return force (mass*linear velocity)
 **/
virtual double getMotionForce ();

/**
 *Calculates the linear force between two components at the moment of calling this function.
 *@return force (masses*linear velocity of both atoms)
 **/
virtual double getCollisionForce ( AtomComponent* collAtom );

/**
 *Tests if the conditions for fusion of two atoms are fulfilled.
 *@param Geom reference to the first atom
 *@param Geom reference to the second atom
 *@return true if fulfilled false if not
 **/
virtual bool fusionCondition ( dGeomID o1 , dGeomID o2 );

/**
 *Tests if the conditions for fission of two atoms are fulfilled.
 *@param Geom reference to the first atom
 *@param Geom reference to the second atom
 *@return true if fulfilled false if not
 **/
virtual bool fissionCondition ( dGeomID o1 , dGeomID o2 );

/**
 *Test which collisions should not be handled.
 *@param Geom reference to the first atom
 *@param Geom reference to the second atom
 *@return true if fulfilled false if not
 **/
virtual bool collisionExclusionCondition ( dGeomID o1 , dGeomID o2 );

/**
 *fuses this AtomComponent with an other one
 *@param the AtomComponent, to fuse with
 *@retuen true if the fusion was successfull, false if not
 **/
virtual bool fusion ( AtomComponent* atom_to_fuse );

/**
 *fissions a AtomComponent from this AtomComponent
 *@param the AtomComponent, to fission
 *@retuen true if the fission was successfull, false if not
 **/
virtual bool fission ( /*AtomComponent* atom_to_fission*/ );

/**
 *This method adds an existing Component as a subcomponent to this component, overwriting the function from the component base class.
 *@param subcomponent to add
 *@param reference to external created joint, which connects both components
 **/
//virtual void addSubcomponent ( Component* newsubcomponent , Joint* newconnectingjoint );
     
};


}
#endif
