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

#include "component.h"

namespace lpzrobots
{

/*****************************************************************************/
/* Component                                                                 */
/*****************************************************************************/
    
    Component::Component ( const OdeHandle &odeHandle, const OsgHandle &osgHandle , const ComponentConf& conf = getDefaultConf () ) : OdeRobot ( odeHandle, osgHandle , std::string ( "Component" ) , std::string ( "component" ) ) , conf ( conf )
    {
	originComponent = this;
	directOriginComponent = this;
    }

    Component::~Component ()
    {
    }

    bool 	Component::collisionCallback (void *data, dGeomID o1, dGeomID o2)
    {
	return false;
    }

    void 	Component::doInternalStuff (const GlobalData &globalData)
    {

    }

    void Component::resetMotorsRecursive ()
    {
	for ( int n = 0; ( (unsigned int) n < connection.size() ); n++ ) //garants that there is no wrong memory access
	{
	    connection[n].joint->setParam ( dParamVel , 0 ); // set velocity
	    connection[n].joint->setParam ( dParamFMax ,0 ); // set maximal force

	    if ( ( dJointGetType ( connection[n].joint->getJoint () ) == dJointTypeHinge2 ) || ( dJointGetType ( connection[n].joint->getJoint () ) == dJointTypeUniversal ) )
	    {
		connection[n].joint->setParam ( dParamVel2 , 0 );  // set velocity2
		connection[n].joint->setParam ( dParamFMax2 , 0 ); // set maximal force2
	    }

	    connection[n].subcomponent->resetMotorsRecursive ();
	}
    }

    osg::Vec3 Component::getPositionbetweenComponents ( Component* component )
    {
	osg::Vec3 posi1 = getMainPrimitive ()->getPosition ();
	osg::Vec3 posi2 = component->getMainPrimitive ()->getPosition ();
	osg::Vec3 anchor = osg::Vec3 ( posi1[0] + ( posi2[0] - posi1[0])/2 , posi1[1] + ( posi2[1] - posi1[1])/2 , posi1[2] + ( posi2[2] - posi1[2])/2 );

	return anchor;
    }

    int Component::getNumberSubcomponents ()
    {
	int counter = 0;
//	componentConnection con = connection.back ();
	for ( unsigned int n = 0; n < connection.size (); n++ )
	    if ( connection[n].softlink == false )
		counter++;

	return counter;
//	return connection.size ();
    }

    int Component::getNumberSubcomponentsAll ()
    {
	int size = 0;

	size += getNumberSubcomponents ();

	for ( unsigned int n = 0; n < connection.size (); n++ )
	{
	    if ( connection[n].softlink == false )
		size += connection[n].subcomponent->getNumberSubcomponentsAll ();
	}
	return size;
    }

    void Component::addSubcomponent ( Component* newsubcomponent , Joint* newconnectingjoint , bool softlink )
    {
//	cout<<"addSubcomponent reached\n";
	componentConnection newconnection;
	newconnection.subcomponent = newsubcomponent;
	newconnection.joint = newconnectingjoint;

	newconnection.softlink = softlink;
	newconnection.data = NULL;

	//sets the origin; if it is NULL if a subcomponent i added, no subcomponent has ever been added here; now it becomes the origin of an subcomponent structure, so its origin is set to itsself
	    if ( softlink == false )
	    {
		newconnection.subcomponent->originComponent = originComponent;
		newconnection.subcomponent->directOriginComponent = this;
	    }
	
	connection.push_back ( newconnection );
//	cout<<"addSubcomponent ended\n";
    }

    Component* Component::removeSubcomponent ( int n )
    {
	if ( (int) connection.size () > 0 && (int) connection.size () > n )
	{
	    Component* tmpcomponent;

	    vector <componentConnection>::iterator eraseiterator;
	    eraseiterator = connection.begin () + n;

	    //move origins
	    tmpcomponent = connection[n].subcomponent;

	    //updates the references of origin within the tree structure !!!BUT ONLY IF IT WAS NO SOFTLINK, WHICH WAS DESTROYED
	    if ( connection[n].softlink == false )
		tmpcomponent->updateOriginsRecursive ( tmpcomponent );


	    //deleting the extra data pointer
	    if (connection[n].data != NULL )
	    {
	        free ( connection[n].data );
		connection[n].data = NULL;
	    }
	    else
		cout<<"tries to delete void* data, but no pointer is set\n";

	    //deleting the joint
	    if ( connection[n].joint != NULL )
	    {
		delete ( connection[n].joint );
		connection[n].joint = NULL;
	    }
	    else
		cout<<"tries to delete Joint* joint, but no joint - pointer is set\n";

//	    cout<<"size before remove: "<<connection.size()<<"\n";
	    connection.erase ( eraseiterator );
//	    cout<<"size after remove: "<<connection.size()<<"\n";


	    return tmpcomponent;
	}
	return NULL;
    }

    Component* Component::removeSubcomponent ( Component* removedsubcomponent )
    {
	Component* tmpcomponent = NULL;

	vector<componentConnection>::iterator it = connection.begin ();
	for ( unsigned int n = 0; n < connection.size (); n++ )
	{
	    if ( connection[n].subcomponent == removedsubcomponent )
	    {
		//move origins
		tmpcomponent = connection[n].subcomponent;

		//updates the references of origin within the tree structure !!!BUT ONLY IF IT WAS NO SOFTLINK, WHICH WAS DESTROYED
		if ( connection[n].softlink == false )
		    tmpcomponent->updateOriginsRecursive ( tmpcomponent );


		//deleting the extra data pointer
		if (connection [n].data != NULL )
		{
		    free ( connection[n].data );
		    connection[n].data = NULL;
		}
		else
		    cout<<"tries to delete void* data, but no pointer is set\n";

		//deleting the joint
		if ( connection[n].joint != NULL )
		{
		    delete ( connection[n].joint );
		    connection[n].joint = NULL;
		}
		else
		    cout<<"tries to delete Joint* joint, but no joint - pointer is set\n";

//		cout<<"size before remove: "<<connection.size()<<"\n";
		connection.erase ( it );
//		cout<<"size after remove: "<<connection.size()<<"\n";



		break;
	    }
	    it++;
	}

	return tmpcomponent;

    }

    void Component::removeAllSubcomponentsRecursive ()
    {
	for ( unsigned int n = 0; n < connection.size (); n ++ )
	{
	    if ( connection[n].softlink == false )
		connection[n].subcomponent->removeAllSubcomponentsRecursive ();

	    removeSubcomponent ( n );
	}

    }

    void Component::updateOriginsRecursive ( Component* parent )
    {
	if ( parent != this )
	{
	    directOriginComponent = parent;
	    originComponent = parent->originComponent;
	}
	else
	{
	    directOriginComponent = this;
	    originComponent = this;
	}
	
	
	for ( unsigned int n = 0; n < connection.size (); n++ )
	{
	    connection[n].subcomponent->updateOriginsRecursive ( this );
	}
    }

    void Component::removeSoftlinksRecursive ()
    {
	for ( unsigned int n = 0; n < connection.size (); n ++ )
	{
	    if ( connection[n].softlink == true )
	    {
		if ( connection.size () > n )
		{
	    
		    vector <componentConnection>::iterator eraseiterator;
		    eraseiterator = connection.begin () + n;
		    
		    //deleting the extra data pointer
		    if (connection [n].data != NULL )
		    {
			free ( connection[n].data );
			connection[n].data = NULL;
		    }
		    else
			cout<<"tries to delete void* data, but no pointer is set\n";
		    
		    //deleting the joint
		    if ( connection[n].joint != NULL )
		    {
			delete ( connection[n].joint );
			connection[n].joint = NULL;
		    }
		    else
			cout<<"tries to delete Joint* joint, but no joint - pointer is set\n";
		    
		    connection.erase ( eraseiterator );
	    
		}
	    }
	    else
		connection[n].subcomponent->removeSoftlinksRecursive ();
	}
    }

    bool Component::hasSubcomponent ( Component* subcomp )
    {
	for ( unsigned int n = 0; n < connection.size (); n++ )
	{
//	    if ( connection[n].softlink == false )
		if ( subcomp == connection[n].subcomponent )
		    return true;
	}
	return false;
    }

/*    bool Component::hasSubcomponentAll ( Component* subcomp )
    {
	if ( hasSubcomponent ( subcomp ) == true )
	    return true;
	
	for ( unsigned int n = 0; n < connection.size (); n++ )
	{
	    if ( connection[n].softlink == false )
	    {
		if ( connection[n].subcomponent->hasSubcomponentAll ( subcomp ) == true )
		    return true;
	    }
	}
	return false;
    }
*/

    bool Component::isComponentConnected ( Component* connectedComp )
    {
	if ( originComponent == connectedComp->originComponent )
	    return true;
	else
	    return false;
    }

    Component::componentConnection  Component::getConnection ( int connectionnumber )
    {
	return connection[connectionnumber];
    }

/*    bool Component::setSoftlink ( unsigned int position , bool state )
    {
	if ( connection.size () > position )
	{
	    connection[position].softlink = state;
	    return true;
	}
	else
	    return false;
    }
*/

    Component* Component::getBestDivideComponent ( double targetrelation , int maxsize , Component* currentBestDivideComponent )
    {
	cout<<"target relation: "<<targetrelation<<"\n";
	cout<<"maxsize: "<<maxsize<<"\n";


	double a;

	for ( unsigned int n = 0; n < connection.size (); n++ )
	{
	    if ( connection[n].softlink == false )
	    {
		//calculation the efficency of dividing the structure here (connection n of this)
		a = ( abs ( ( (connection[n].subcomponent->getNumberSubcomponentsAll () +1 )/((double) maxsize) - targetrelation)*1000000)/1000000.0 );
		//1000000 because abs does only work with integers

		cout<<"-----------------------------------------------component: "<<this<<"\n";
		cout<<"connection "<<n<<"     relation= "<<a<<" \n";
		//comparing the dividing efficience to the best efficience up till now, if better it becomes the new best
		if ( currentBestDivideComponent != NULL )
		{
		    if ( a < ( abs ( ( ( currentBestDivideComponent->getNumberSubcomponentsAll () + 1 )/ ((double) maxsize)  - targetrelation )*1000000)/1000000.0 ) )
		    {
			currentBestDivideComponent = connection[n].subcomponent;
			cout<<"new best hit\n";
		    }
		}
		else
		{
		    currentBestDivideComponent = connection[n].subcomponent;
		    cout<<"first best hit\n";
		}
		
		//recusive call for the subcomponents
		currentBestDivideComponent = connection[n].subcomponent->getBestDivideComponent ( targetrelation , maxsize , currentBestDivideComponent );
	    }
	}

	return currentBestDivideComponent;
    }

}

