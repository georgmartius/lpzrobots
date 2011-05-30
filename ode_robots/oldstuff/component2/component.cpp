
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
 *   Revision 1.1  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.16  2006/11/30 08:51:39  robot8
 *   -update of the evolution projekt
 *   -fitness changed
 *   -replication changed
 *   -added copy function
 *
 *   Revision 1.15  2006/10/20 13:52:32  robot8
 *   -update of the evolution projekt
 *   -only changed some parameter
 *
 *   Revision 1.14  2006/10/10 07:49:39  robot8
 *   -update of the evolution projekt
 *   -only changed some parameter
 *
 *   Revision 1.13  2006/09/11 12:00:51  martius
 *   removed accidental "ma" at beginning of file
 *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include "component.h"

using namespace std;

namespace lpzrobots
{

/*****************************************************************************/
/* Component                                                                 */
/*****************************************************************************/
    
    Component::Component ( const OdeHandle &odeHandle, const OsgHandle &osgHandle , const ComponentConf& conf = getDefaultConf () ) : OdeRobot ( odeHandle, osgHandle , std::string ( "Component" ) , std::string ( "component" ) ) , conf ( conf )
    {
	originComponent = this;
	directOriginComponent = this;
	connection.clear ();
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

    double Component::getDistanceToComponent ( Component* comp )
    {
	double x,y,z;
	x = getPosition ().x - comp->getPosition ().x;
	y = getPosition ().y - comp->getPosition ().y;
	z = getPosition ().z - comp->getPosition ().z;

	return sqrt ( pow ( x , 2 ) + pow ( y , 2 ) + pow ( z , 2 ) );
    }


    int Component::getSensorNumber ()
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
	     
	    //recursive sensor-counting for all subcomponents
	    if ( connection[n].softlink == false )
		motors += connection[n].subcomponent->getMotorNumber ();
	}

	return motors;
    }



    int Component::getSensors ( sensor *sensors , int sensornumber )
    {
	int sensorcounter = 0;
	

//	if ( sensornumber <= getSensorNumber () )
//	{
	    //sensor values of this component
	  for ( int n = 0; n < getNumberSubcomponents (); n++ ){
	    Joint* j = connection[n].joint;
	    sensorcounter += j->getPositions(sensors);
	  }
	    

	  //sensor values of all subcomponents and their robots
	  for ( int n = 0; n < getNumberSubcomponents (); n++ )
	    {
	      if ( connection[n].softlink == false )
		sensorcounter += connection[n].subcomponent->getSensors (&sensors[sensorcounter] , 
									 connection[n].subcomponent->getSensorNumber () );
	    }	  
//	}
	return sensorcounter;
    }

    void Component::setMotors ( const motor *motors , int motornumber )
    {
	int motorcounter = 0;
	motor* tmpmotors;

	for ( int n = 0; ( (unsigned int) n < connection.size() ) && ( n < motornumber ); n++ ) //garants that there is no wrong memory access
	{
	    connection[n].joint->setParam ( dParamVel , motors[n]*conf.speed ); // set velocity
	    connection[n].joint->setParam ( dParamFMax ,conf.max_force );       // set maximal force
	    motorcounter++;
    
	    if ( connection[n].joint->getNumberAxes() == 2 )
	    {
		connection[n].joint->setParam ( dParamVel2 , motors[n]*conf.speed ); // set velocity2
		connection[n].joint->setParam ( dParamFMax2 ,conf.max_force );       // set maximal force2
		motorcounter++;
	    }

	}

	for ( int n = 0; ( (unsigned int) n < connection.size() ) && ( n < motornumber ); n++ ) //garants that there is no wrong memory access
	{
	    if ( connection[n].softlink == false )
	    {
		tmpmotors = (motor*) &motors[motorcounter]; //the pointer for the new array
		connection[n].subcomponent->setMotors ( tmpmotors , motornumber - motorcounter );
		motorcounter += connection[n].subcomponent->getMotorNumber ();//the start of the array is shifted by the number of used array elements
	    }

	}
    }



    void Component::resetMotorsRecursive ()
    {
	for ( int n = 0; ( (unsigned int) n < connection.size() ); n++ )
	{
	    connection[n].joint->setParam ( dParamVel , 0 ); // set velocity
	    connection[n].joint->setParam ( dParamFMax ,0 ); // set maximal force

	    if ( connection[n].joint->getNumberAxes() == 2 )
	    {
		connection[n].joint->setParam ( dParamVel2 , 0 );  // set velocity2
		connection[n].joint->setParam ( dParamFMax2 , 0 ); // set maximal force2
	    }

	    if ( connection[n].softlink == false )
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

/*************************connection part***************************/
//	cout<<"addSubcomponent reached\n";
	componentConnection newconnection;
	newconnection.subcomponent = newsubcomponent;
	newconnection.joint = newconnectingjoint;

	newconnection.softlink = softlink;
	newconnection.data = NULL;

	//sets the origin; if it is NULL if a subcomponent i added, no subcomponent has ever been added here; now it becomes the origin of an subcomponent structure, so its origin is set to itsself
	if ( softlink == false )
	{
//origin update has to be necessary because the whole structre could have changed because of possible some restructuring functions
	    newconnection.subcomponent->updateOriginsRecursive ( this );

//		newconnection.subcomponent->originComponent = originComponent;
//		newconnection.subcomponent->directOriginComponent = this;
	}
	
	connection.push_back ( newconnection );
//	cout<<"addSubcomponent ended\n";

/*************************backward reference part***************************/
	
	if ( softlink == true )
	    connection.back ().subcomponent->backwardreference.push_back ( this );


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
	    else
	    {

		vector <Component*>::iterator it;
		for ( it = tmpcomponent->backwardreference.begin (); *it != this; )
		    it++;

		
		//removing the backward reference in the subcomponent
		tmpcomponent->backwardreference.erase ( it );
	    }


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

	    connection.erase ( eraseiterator );

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
		else
		{
		    vector <Component*>::iterator it2;
		    for ( it2 = tmpcomponent->backwardreference.begin (); *it2!= this; it2++ )
			;
		    //removing the backward reference in the subcomponent
		    tmpcomponent->backwardreference.erase ( it2);
		    cout<<"backwardreference remove delting\n";
		}


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

		connection.erase ( it );
		


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

	}

	for ( unsigned int n = 0 ; n <  backwardreference.size (); n++ )
	{
	    backwardreference[0]->removeSubcomponent ( this );//always the first will be removed, until no elements are left in the vector   
	}

	for ( unsigned int n = 0; n < connection.size (); n ++ )
	{
	    removeSubcomponent ( 0 ); //always the first will be removed, until no elements are left in the vector
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
	    if ( connection[n].softlink == false )
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

	backwardreference.clear ();
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

    Component::componentConnection*  Component::getConnection ( unsigned int connectionnumber )
    {
	if ( connectionnumber < connection.size () )
	    return &connection[connectionnumber];
	else
	    return NULL;
    }

    Component::componentConnection*  Component::getConnection ( Component* targetcomponent )
    {
	for ( unsigned int n = 0; n < connection.size (); n++ )
	{
	    if ( connection[n].subcomponent == targetcomponent )
		return &connection[n];
	}

	return NULL;
    }

    bool Component::setSoftlink ( unsigned int position , bool state )
    {
	if ( connection.size () > position )
	{
	    connection[position].softlink = state;
	    return true;
	}
	else
	    return false;
    }


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
		a = ( fabs ( ( (connection[n].subcomponent->getNumberSubcomponentsAll () +1 )/((double) maxsize) - targetrelation)*1000000)/1000000.0 );
		//1000000 because abs does only work with integers

		cout<<"-----------------------------------------------component: "<<this<<"\n";
		cout<<"connection "<<n<<"     relation= "<<a<<" \n";
		//comparing the dividing efficience to the best efficience up till now, if better it becomes the new best
		if ( currentBestDivideComponent != NULL )
		{
		    if ( a < ( fabs ( ( ( currentBestDivideComponent->getNumberSubcomponentsAll () + 1 )/ ((double) maxsize)  - targetrelation )*1000000)/1000000.0 ) )
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



};

