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

    int SimpleComponent::getSensorNumber ()
    {
	int sensors = 0;   
	//recursive sensor-counting for all subcomponents

	for ( int n = 0; n < getNumberSubcomponents (); n++ )
	{
	    Joint* j = connection[n].joint;
	    sensors += j->getNumberAxes();

    	    //recursive sensor-counting for all subcomponents
	    if ( connection[n].softlink == false )
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
	    if ( connection[n].softlink == false )
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
	for ( int n = 0; n < getNumberSubcomponents (); n++ )
	{
//		if ( dynamic_cast <RobotComponent*> ( connection[n].subcomponent )  != NULL ) //if the pointer could be casted to RobotComponent, then it is a RobotComponent

	    if ( connection[n].subcomponent->collisionCallback ( data , o1 , o2 ) )
		return true; // exit if collision was treated by a robot/component
	}
	return false; //a simpleComponent does never handle collisions itself, it uses the standard collisionCallback of the simulation
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
}

