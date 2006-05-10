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

#include "simplecomponent.h"

namespace lpzrobots
{

/*****************************************************************************/
/* SimpleComponent                                                                 */
/*****************************************************************************/
    
    SimpleComponent::SimpleComponent ( const OdeHandle &odeHandle, const OsgHandle &osgHandle , const ComponentConf& conf = Component::getDefaultConf () ) : Component ( odeHandle, osgHandle , conf )
    {
	simplePrimitive = NULL;
    
    }

    SimpleComponent::~SimpleComponent ()
    {
    }

    int SimpleComponent::getSensors ( sensor *sensors , int sensornumber )
    {
	int sensorcounter = 0;
	

	if ( sensornumber == getSensorNumber () )
	{
	    //sensor values of this component
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
		    if ( ( dJointGetType ( connection[n].joint->getJoint () ) == dJointTypeHinge2 ) || ( dJointGetType ( connection[n].joint->getJoint () ) == dJointTypeUniversal ) )
		    {
			sensors[n] = ((TwoAxisJoint*) connection[n].joint)->getPosition2 ();
			sensorcounter++;
		    }
		}

	    //sensor values of all subcomponents and their robots
	    for ( int n = 0; n < getNumberSubcomponents (); n++ )
		sensorcounter += connection[n].subcomponent->getSensors ( &sensors[sensorcounter] , connection[n].subcomponent->getSensorNumber () );

	}

	return sensorcounter;
    }

    void SimpleComponent::setMotors ( const motor *motors , int motornumber )
    {
	int motorcounter = 0;
	motor* tmpmotors;

	for ( int n = 0; ( (unsigned int) n < connection.size() ) && ( n < motornumber ); n++ ) //garants that there is no wrong memory access
	{
	    connection[n].joint->setParam ( dParamVel , motors[n]*conf.speed ); // set velocity
	    connection[n].joint->setParam ( dParamFMax ,conf.max_force );       // set maximal force
	    motorcounter++;

	    if ( ( dJointGetType ( connection[n].joint->getJoint () ) == dJointTypeHinge2 ) || ( dJointGetType ( connection[n].joint->getJoint () ) == dJointTypeUniversal ) )
	    {
		connection[n].joint->setParam ( dParamVel2 , motors[n]*conf.speed ); // set velocity2
		connection[n].joint->setParam ( dParamFMax2 ,conf.max_force );       // set maximal force2
		motorcounter++;
	    }

	}

	for ( int n = 0; ( (unsigned int) n < connection.size() ) && ( n < motornumber ); n++ ) //garants that there is no wrong memory access
	{
	    tmpmotors = (motor*) &motors[motorcounter]; //the pointer for the new array
	    connection[n].subcomponent->setMotors ( tmpmotors , motornumber - motorcounter );
	    motorcounter += connection[n].subcomponent->getMotorNumber ();//the start of the array is shifted by the number of used array elements

	}
    }

    int SimpleComponent::getSensorNumber ()
    {
	int sensors = 0;   

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


    int SimpleComponent::getMotorNumber ()
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
	     
	    //recursive sensor-counting for all subcomponents
	    motors += connection[n].subcomponent->getMotorNumber ();
	}

	return motors;
    }

    void SimpleComponent::update ()
    {
	//there is a simplePrimitive, and it is updated
	if ( simplePrimitive != NULL )
	    simplePrimitive->update ();
	//all subcomponents and joints also are updated
	for ( int n = 0; n < getNumberSubcomponents (); n++ )
	{
	    connection[n].joint->update ();
	    connection[n].subcomponent->update ();
	}
    }

    void SimpleComponent::place ( const Pos &pos )
    {
	Position newpos;

	if ( simplePrimitive != NULL) //there is a simplePrimitive, and its position is updated
	{
	    simplePrimitive->setPosition ( osg::Vec3 ( ((Pos)pos).toPosition().x , ((Pos)pos).toPosition().y , ((Pos)pos).toPosition().z ) );
	}

	for ( int n = 0; n < getNumberSubcomponents (); n++ )
	{
	    newpos = connection[n].subcomponent->getPosition () - ((Pos)pos).toPosition ();
	    connection[n].subcomponent->place ( *(new Pos ( newpos )) );
	}
    }

    void SimpleComponent::place (const osg::Matrix &pose)
    {

    }

    bool SimpleComponent::collisionCallback (void *data, dGeomID o1, dGeomID o2)
    {
	return false;
    }

    void SimpleComponent::doInternalStuff (const GlobalData &globalData)
    {

    }

    Position SimpleComponent::getPosition () const
    {
	    osg::Vec3 position = simplePrimitive->getPosition();
	    return Position ( position[0], position[1] , position[2] );
    }

    osg::Vec3 SimpleComponent::getPositionbetweenComponents ( Component* component )
    {
	osg::Vec3 posi1 = getMainPrimitive ()->getPosition ();
	osg::Vec3 posi2 = component->getMainPrimitive ()->getPosition ();
	osg::Vec3 anchor = osg::Vec3 ( posi1[0] + ( posi2[0] - posi1[0])/2 , posi1[1] + ( posi2[1] - posi1[1])/2 , posi1[2] + ( posi2[2] - posi1[2])/2 );

	return anchor;
    }


    bool SimpleComponent::setSimplePrimitive ( Primitive* newprimitive )
    {
	if ( simplePrimitive != NULL )
	{
	    simplePrimitive = newprimitive;
	    return true;
	}
	else
	{
	    simplePrimitive = newprimitive;
	    return false;
	}
    }

    Primitive* SimpleComponent::getMainPrimitive () const
    {
	    if ( simplePrimitive != NULL )
		return simplePrimitive;
	    else
		return NULL;
    }

    int SimpleComponent::getNumberSubcomponents ()
    {
	return connection.size ();
    }

    int SimpleComponent::getNumberSubcomponentsAll ()
    {
	int size = 0;

	size += getNumberSubcomponents ();

	for ( unsigned int n = 0; n < connection.size (); n++ )
	{
	    size += connection[n].subcomponent->getNumberSubcomponentsAll ();
	}
	return size;
    }


    void SimpleComponent::addSubcomponent ( Component* newsubcomponent , Joint* newconnectingjoint )
    {
	componentConnection newconnection;
	newconnection.subcomponent = newsubcomponent;
	newconnection.joint = newconnectingjoint;

	connection.push_back ( newconnection );
    }

    Component* SimpleComponent::removeSubcomponent ( int n )
    {
	Component* tmpcomponent;

	vector <componentConnection>::iterator eraseiterator;
	eraseiterator = connection.begin () + n;
	tmpcomponent = connection[n].subcomponent;
	connection.erase ( eraseiterator );
	return tmpcomponent;
    }


    Component* SimpleComponent::removeSubcomponent ( Component* removedsubcomponent )
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

