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
 *                                                                         *
 *   $Log$
 *   Revision 1.11  2006-09-11 12:01:31  martius
 *   *** empty log message ***
 *
 *                                                                         *
 ***************************************************************************/

#ifndef _component_h
#define _component_h

#include <typeinfo>
#include <vector>
#include <list>

#include <ode/ode.h>
#include <math.h>

#include "oderobot.h"
#include "joint.h"

namespace lpzrobots
{
  typedef struct _ComponentConf
  {
    bool completesensormode; //if true, the controler of the Component also gets the sensor values of the robot of the component
    bool completemotormode; //if true, the controler of the component also controls the robot of the component
    double max_force;
    double speed;
  } ComponentConf;

  /**
   * This is the abstract base class of the component system. A component includes the routines for managing sensor and motor values from a controller.
   * With a bunch of components it is possible to create a tree like structure. There is always a root structure.
   * Components could be used as normal robot objects, so they can get motor and sensor values. These values are shared with all components in the tree structure of a component.
   * Between the differnt objects of the component class could exist connections, which are physical represented by Ode-Joints.
   * The components use the motor values they get, to control the joints between them.
   * From these joints sensor values are read out, which where send with the getSensor () function like a normal robot does.
   * But the arrray of sensor values comes from all components within the structure beneth the component, from which the function was called.
   * You could say that each component is a non physical shell around a physical object like a primitive or a robot in classical meaning of this project.
   * These shells build a connection between the physical parts and control these connections, which could be also physical existend.
   */
  class Component : public OdeRobot
    {
    public:
      ComponentConf conf;

    public:

      /**
       *This is the structure of one connection between two components.
       **/
      typedef struct
      {
	Component* subcomponent;
	Joint* joint;
	bool softlink; //if true the connection ends the recursion, false = normal
	void *data;
      } componentConnection;

      std::vector <componentConnection> connection;
      std::vector <Component*> backwardreference;

    public: 
      Component* originComponent;
      Component* directOriginComponent;

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
       *Returns number of sensors; recursivly adding of the number of sensors all subcomponents and the robots of all Subcomponents.
       *@return number of sensors of the component and all its subcomponents
       **/
      virtual int 	getSensorNumber ();

      /**
       *returns number of motors; recursivly adding of the number of sensors all subcomponents; at the moment only counts Hinge-, Slider-, Hinge2 and Universal-Joints; The Motor-Numbers of the robots of the Components is not counted.
       *@return number of motors of the component and all its subcomponents
       **/
      virtual int 	getMotorNumber (); 

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

      /**
       *This sets all motors in the component structure to zero, starting from this component.
       **/
      virtual void    resetMotorsRecursive ( );

      //virtual int 	getSensorNumber () = 0; //returns number of sensors; recursivly adding of the number of sensors all subcomponents and the robots of all Subcomponents.

      //virtual int 	getMotorNumber () = 0; //returns number of motors; recursivly adding of the number of sensors all subcomponents; at the moment only counts Hinge-, Slider-, Hinge2 and Universal-Joints; The Motor-Numbers of the robots of the Components is not counted.

      virtual void 	update () = 0;//update the OSG notes here; update of the underlying robot or Primitive and recursive update of all Components in the Connection-vector

      virtual void 	place (const Pos &pos) = 0;//sets the vehicle to position pos - desired position of the robot; the first component is seen as center of the robot, on which the position pos refers; also recursive place of all subComponents

      virtual bool 	collisionCallback (void *data, dGeomID o1, dGeomID o2);// checks for internal collisions and treats them.; should do nothing, because there should not be any ode-objects belonging to the component, which are not handled elsewhere....and what is with Primitives? are they automaticaly handled?

      virtual void 	doInternalStuff (const GlobalData &globalData);// this function is called in each timestep.; maybee usefull

      // virtual void 	setColor (const Color &col); 	sets color of the robot; not nessecary

      virtual Position getPosition () const = 0; //returns position of the object; relates to the robot or Primitive belonging to the component

      /**
       *Gets the distance between two components.
       *@return distance in space
       **/
      virtual double getDistanceToComponent ( Component* comp );

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
      virtual void addSubcomponent ( Component* newsubcomponent , Joint* newconnectingjoint , bool softlink );

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
       *This removes all subcomponents of THIS component, and all their subcomponents, till the whole structure is destroyed.
       **/
      //virtual void removeAllSubcomponentsRecursive ();

      /**
       *This updates the origin references within the component tree. If this is a removed subcomponent for examble, then parent should be this itself, so it is the top of the tree.
       *@param the component wich is the top of the tree structure, could also be this component itself
       **/
      virtual void updateOriginsRecursive ( Component* parent );

      /**
       *This removes all softlinks of the structure that has THIS as his origin.
       **/
      virtual void removeSoftlinksRecursive ();

      /**
       *This method looks if a special component is a subcomponent of this component. Only direct subcomponents are registrated.
       *Components which are only connected by softlink are a connection here as well.
       *@param the component, which could be a subcomponent of this component
       *@return true if it is a subcomponent, false if not
       **/
      virtual bool hasSubcomponent ( Component* subcomp );


      /**
       *This method looks if a special component is a subcomponent of this component and all its recursive subcomponents.
       *Softlinks count as connected, but are not recursivly counted.
       *@param the component, which could be a subcomponent
       *@return true if it is a subcomponent, false if not
       **/
      //virtual bool hasSubcomponentAll ( Component* subcomp );

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
      virtual componentConnection* getConnection ( unsigned int connectionnumber );

      /**
       *This returns the connection that connects to the target component.
       *@param the reference to the component which is subcomponent in the searched connection
       *@return the reference of connection element
       **/
      virtual componentConnection* getConnection ( Component* targetcomponent );

      /**
       *Sets the connection between the component and one of its subcomponents to be a softlink. That means that the recusion for this branch stops here.
       *@param number of the subcomponent in the subcomponent list of the component
       *@param true = connection becomes a softlink
       **/
      virtual bool setSoftlink ( unsigned int position , bool state );

      /**
       *This divides the component structure, following this component into two seperate component structures
       *@param the relation between the two new component structures, if it is 1/2 the structures would have the same number of components or only have a difference by one if the whole structures has an odd number of components
       *@param maximum size of the whole Component structure of the first call of this function
       *@param the best component for dividing at the moment of calling this function; it should be NULL for the first call of the function, so that no best component is set up till now
       *@return the best Componet for dividing at the moment the function is finished
       */
      virtual Component* getBestDivideComponent ( double targetrelation , int maxsize , Component* currentBestDivideComponent );
     
    };


}
#endif
