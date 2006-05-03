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
 ***************************************************************************/

#include "component.h"

namespace lpzrobots
{

/*****************************************************************************/
/* Component                                                                 */
/*****************************************************************************/
    
    Component::Component ( const OdeHandle &odeHandle, const OsgHandle &osgHandle , const ComponentConf& conf = getDefaultConf () ) : OdeRobot ( odeHandle, osgHandle , "Component" ) , conf ( conf )
    {
	robot = NULL;
	simplePrimitive = NULL;
    
    }

    Component::~Component ()
    {
    }

    int Component::getSensors ( sensor *sensors , int sensornumber )
    {
	int sensorcounter = 0;
	

	if ( sensornumber == getSensorNumber () )
	{
	    //sensor values of this component, and its robot
	    for ( int n = 0; n < getNumberSubcomponents (); n++ )
		//Fixed- and Ball-Joint-Classes do not have the getPosition-function
		if ( ( dJointGetType ( connection[n].joint->getJoint () ) == dJointTypeFixed ) || ( dJointGetType ( connection[n].joint->getJoint () ) == dJointTypeBall ) )
		{
		    //nothing is done
		}
		else //now all other joints, which should be normaly used, are treated; they are all subclasses of the OneAxisJoint-class
		{
		    sensors[n] = ((OneAxisJoint*) connection[n].joint)->getPosition1 ();
		    sensorcounter++;
		}


	    if ( conf.completesensormode == true && robot != NULL )
	    {
		robot->getSensors ( &sensors[sensorcounter] , sensornumber - ( sensorcounter ) );
		sensorcounter += robot->getSensorNumber ();
	    }

	    //sensor values of all subcomponents and their robots
	    for ( int n = 0; n < getNumberSubcomponents (); n++ )
		sensorcounter += connection[n].subcomponent->getSensors ( &sensors[sensorcounter] , connection[n].subcomponent->getSensorNumber () );

	}

	return sensorcounter;
    }

    void Component::setMotors ( const motor *motors , int motornumber )
    {
	int motorcounter = 0;
	motor* tmpmotors;

	for ( int n = 0; ( n < connection.size() ) && ( n < motornumber ); n++ ) //garants that there is no wrong memory access
	{
	    connection[n].joint->setParam ( dParamVel2 , motors[n]*conf.speed ); // set velocity
	    connection[n].joint->setParam ( dParamFMax2 ,conf.max_force );       // set maximal force
	}
	motorcounter += getNumberSubcomponents (); //the start of the array is shifted by the number of used array elements

	//setMotors for the robot, if it exists
	if ( conf.completemotormode == true && robot != NULL )
	{
	    robot->setMotors ( &motors[motorcounter] , motornumber - motorcounter );
	    motorcounter += robot->getMotorNumber ();
	}

	for ( int n = 0; ( n < connection.size() ) && ( n < motornumber ); n++ ) //garants that there is no wrong memory access
	{
	    tmpmotors = (motor*) &motors[motorcounter]; //the pointer for the new array
	    connection[n].subcomponent->setMotors ( tmpmotors , motornumber - motorcounter );
	    motorcounter += connection[n].subcomponent->getMotorNumber ();//the start of the array is shifted by the number of used array elements

	}
    }

    int Component::getSensorNumber ()
    {
	int sensors;   

	//if the sensor values should be used, and a robot is there, the robot-sensor number is added
	    if ( conf.completesensormode == true && robot != NULL )
		sensors = robot->getSensorNumber ();
	    //recursive sensor-counting for all subcomponents
	
	for ( int n = 0; n < getNumberSubcomponents (); n++ )
	{
	    //counting the sensors by the type of the used joint, coded by ode type, because the joints are created external
	    if ( dJointGetType ( connection[n].joint->getJoint () ) == dJointTypeHinge )
		sensors++;
	    else
		if ( dJointGetType ( connection[n].joint->getJoint () ) == dJointTypeSlider )
		    sensors++;
		else
		    if ( dJointGetType ( connection[n].joint->getJoint () ) == dJointTypeHinge2 )
			sensors = sensors + 2;
		    else
			if ( dJointGetType ( connection[n].joint->getJoint () ) == dJointTypeUniversal )
			    sensors = sensors + 2;
	
    	    //recursive sensor-counting for all subcomponents
	    sensors += connection[n].subcomponent->getSensorNumber ();
	}

	return sensors;
    }


    int Component::getMotorNumber ()
    {
	int motors = 0;

	for ( int n = 0; n < getNumberSubcomponents (); n++ )
	{//counting the motors by the type of the used joint, coded by ode type, because the joints are created external
	    if ( dJointGetType ( connection[n].joint->getJoint () ) == dJointTypeHinge )
		motors++;
	    else
		if ( dJointGetType ( connection[n].joint->getJoint () ) == dJointTypeSlider )
		    motors++;
		else
		    if ( dJointGetType ( connection[n].joint->getJoint () ) == dJointTypeHinge2 )
			motors = motors + 2;
		    else
			if ( dJointGetType ( connection[n].joint->getJoint () ) == dJointTypeUniversal )
			    motors = motors + 2;
	     

	    //if the motor values should be used, and a robot is there, the robot-motor number is added
	    if ( conf.completemotormode == true && robot != NULL )
		motors += robot->getMotorNumber ();
	    //recursive sensor-counting for all subcomponents
	    motors += connection[n].subcomponent->getMotorNumber ();
	}

	return motors;
    }

    void Component::update ()
    {
	//only if there is a robot
	if ( robot != NULL )
	    robot->update ();
	else //there is a simplePrimitive, and it is updated
	    simplePrimitive->update ();
	//all subcomponents and joints also are updated
	for ( int n = 0; n < getNumberSubcomponents (); n++ )
	{
	    connection[n].joint->update ();
	    connection[n].subcomponent->update ();
	}
    }

    void Component::place ( const Pos &pos )
    {
	Position newpos;

	//only if there is a robot
	if ( robot != NULL )
	    robot->place ( pos );
	else //there is a simplePrimitive, and it is updated
	    simplePrimitive->setPosition ( pos );

	for ( int n = 0; n < getNumberSubcomponents (); n++ )
	{
	    newpos = connection[n].subcomponent->getPosition () - ((Pos)pos).toPosition ();
	    connection[n].subcomponent->place ( *(new Pos ( newpos )) );
	}
    }

    void 	Component::place (const osg::Matrix &pose)
    {

    }

    bool 	Component::collisionCallback (void *data, dGeomID o1, dGeomID o2)
    {
	return false;
    }

    void 	Component::doInternalStuff (const GlobalData &globalData)
    {

    }

    Position Component::getPosition () const
    {
	if ( robot == NULL && simplePrimitive != NULL )
	{
	    return Position ( simplePrimitive->getPosition()[0] , simplePrimitive->getPosition()[1] , simplePrimitive->getPosition()[2] );
	}
	else
	    return robot->getPosition ();
    }

    osg::Vec3 Component::getPositionbetweenComponents ( Component* component )
    {
	osg::Vec3 anchor = osg::Vec3 ( getMainPrimitive ()->getPosition ()[0] + ( component->getMainPrimitive ()->getPosition ()[0] - getMainPrimitive ()->getPosition ()[0])/2 ,
					 getMainPrimitive ()->getPosition ()[1] + ( component->getMainPrimitive ()->getPosition ()[1] - getMainPrimitive ()->getPosition ()[1])/2 ,
					 getMainPrimitive ()->getPosition ()[2] + ( component->getMainPrimitive ()->getPosition ()[2] - getMainPrimitive ()->getPosition ()[2])/2 );


	return anchor;
    }


    bool Component::setSimplePrimitive ( Primitive* newprimitive )
    {
	if ( robot == NULL )
	{
	    simplePrimitive = newprimitive;
	    return true;
	}
	else
	    return false;
    }


    bool Component::setRobot ( OdeRobot* newrobot )
    {
	if ( simplePrimitive == NULL )
	{
	    robot = newrobot;
	    return true;
	}
	else
	    return false;
    }

    OdeRobot* Component::getRobot ()
    {
	return robot;
    }

    Primitive* Component::getMainPrimitive () const//overload this in the robot implementation.; should be the main-Primitive from the first componentConnection in the vector
    {
	//if there is a robot belonging to the compoent
	if ( robot != NULL )
	    return robot->getMainPrimitive();
	else //if there is only a Primitive belonging to the component
	    return simplePrimitive;	
    }

    int Component::getNumberSubcomponents ()
    {
	return connection.size ();
    }

    int Component::getNumberSubcomponentsAll ()
    {
	int size = 0;

	size += getNumberSubcomponents ();

	for ( unsigned int n = 0; n < connection.size (); n++ )
	{
	    size += connection[n].subcomponent->getNumberSubcomponentsAll ();
	}
	return size;
    }


    void Component::addSubcomponent ( Component* newsubcomponent , Joint* newconnectingjoint )
    {
	componentConnection newconnection;
	newconnection.subcomponent = newsubcomponent;
	newconnection.joint = newconnectingjoint;

	connection.push_back ( newconnection );
    }

    Component* Component::removeSubcomponent ( int n )
    {
	Component* tmpcomponent;

	vector <componentConnection>::iterator eraseiterator;
	eraseiterator = connection.begin () + n;
	tmpcomponent = connection[n].subcomponent;
	connection.erase ( eraseiterator );
	return tmpcomponent;
    }


    Component* Component::removeSubcomponent ( Component* removedsubcomponent )
    {
	Component* tmpcomponent;

	vector<componentConnection>::iterator it = connection.begin ();
	for ( unsigned int n = 0; n < connection.size (); n++ )
	{
	    it++;
	    if ( connection[n].subcomponent == removedsubcomponent )
	    {
		tmpcomponent = connection[n].subcomponent;
		connection.erase ( it );
		break;
	    }
	}

	return tmpcomponent;

    }

}

