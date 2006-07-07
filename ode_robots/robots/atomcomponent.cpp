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

#include "atomcomponent.h"

namespace lpzrobots
{



/*****************************************************************************/
/* AtomComponent                                                             */
/*****************************************************************************/
    
    AtomComponent::AtomComponent ( const OdeHandle &odeHandle, const OsgHandle &osgHandle , const ComponentConf& conf = Component::getDefaultConf () , const AtomConf& aconf = AtomComponent::getDefaultAtomConf () ) : Component ( odeHandle, osgHandle , conf )
    {
	atomconf = aconf;
	//color settings
	
	Color color2 = Color ( 1 , 1 , 1 );
	color2.alpha() = 0.5;	


	OsgHandle osgHandle_shell = osgHandle.changeColor ( color2 );


	//core
	core = new OSGSphere ( atomconf.core_radius );
	core->init ( osgHandle , OSGPrimitive::Middle );

	//shell
	shell = new Sphere ( atomconf.shell_radius );   
	shell->init ( odeHandle , atomconf.mass , osgHandle_shell , Primitive::Body | Primitive::Geom  | Primitive::Draw );

	dGeomSetData ( shell->getGeom () ,  this );

    }

    AtomComponent::~AtomComponent ()
    {
    }

    int AtomComponent::getSensors ( sensor *sensors , int sensornumber )
    {
	int sensorcounter = 0;

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
			sensors[n+1] = ((TwoAxisJoint*) connection[n].joint)->getPosition2 ();
			sensorcounter++;
		    }
		}

	    //sensor values of all subcomponents and their robots
	    for ( int n = 0; n < getNumberSubcomponents (); n++ )
	    {
		if ( connection[n].softlink == false )
		    sensorcounter += connection[n].subcomponent->getSensors ( &sensors[sensorcounter] , connection[n].subcomponent->getSensorNumber () );
	    }

	//if there are less sensor values than expected (that happens beause the controller size is maximized from the beginning), then
	    for ( ; sensorcounter < sensornumber; ++sensorcounter)
	    {
/*!!!!!!!!!!!!!!*/		sensors[sensorcounter] = 0; //should not be zero, has to be changed in future!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	    }
	    
	return sensorcounter;
    }

    void AtomComponent::setMotors ( const motor *motors , int motornumber )
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
	    if ( connection[n].softlink == false )
	    {
		tmpmotors = (motor*) &motors[motorcounter]; //the pointer for the new array
		connection[n].subcomponent->setMotors ( tmpmotors , motornumber - motorcounter );
		motorcounter += connection[n].subcomponent->getMotorNumber ();//the start of the array is shifted by the number of used array elements
	    }

	}
    }

    int AtomComponent::getSensorNumber ()
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
	    if ( connection[n].softlink == false )
		sensors += connection[n].subcomponent->getSensorNumber ();
	}


	return sensors;
    }


    int AtomComponent::getMotorNumber ()
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

    void AtomComponent::update ()
    {
	    shell->update ();
	    core->setMatrix ( osgPose ( shell->getBody () ) );


	//all subcomponents and joints also are updated
	for ( int n = 0; n < getNumberSubcomponents (); n++ )
	{
	    connection[n].joint->update ();
	    connection[n].subcomponent->update ();
	}
    }

    void AtomComponent::place ( const Pos &pos )
    {
	Position newpos;

 	shell->setPosition ( osg::Vec3 ( ((Pos)pos).toPosition().x , ((Pos)pos).toPosition().y , ((Pos)pos).toPosition().z ) );

	for ( int n = 0; n < getNumberSubcomponents (); n++ )
	{
	    newpos = connection[n].subcomponent->getPosition () - ((Pos)pos).toPosition ();
	    connection[n].subcomponent->place ( *(new Pos ( newpos )) );
	}
    }

    void AtomComponent::place (const osg::Matrix &pose)
    {
	
    }

    bool AtomComponent::collisionCallback (void *data, dGeomID o1, dGeomID o2)
    {

	if ( shellCollision ( o1 , o2 ) == true )
	{

	    if ( dGeomGetClass ( o1 ) == dSphereClass && dGeomGetClass ( o2 ) == dSphereClass ) //only if two atoms colide, all other cases are handled by the default collision handling of the simulations
	    {
		if ( collisionExclusionCondition ( o1 , o2 ) == true )
		{
		    return true; //in this case the collision is ignored
		}
		
		/***************************Fusion***or***Fission*********************************************/
		/*First the decission if there is fusion or fission*/
//		cout<<"Collision with: "<<((AtomComponent*) dGeomGetData ( o2 ))->getCollisionForce ( (AtomComponent*) dGeomGetData ( o1 ) )<<" force\n";
		
		double force = ((AtomComponent*) dGeomGetData ( o2 ))->getCollisionForce ( (AtomComponent*) dGeomGetData ( o1 ) );

		if ( shell->getGeom () == o1 )
		{
		    if ( fusionCondition ( o1 , o2 ) == true )
		    {
			 fusion ( (AtomComponent*) dGeomGetData ( o2 ) ); //FUSION is called;
			 return true;
		    }

		    if ( fissionCondition ( o1 , o2 , force ) == true )
		    {
			fission ( force ); //FISSION is called
			return true;
		    }

		    //if no fusion and no fission dit happen
		    return false;
		}
		else
		    if ( shell->getGeom () == o2 )
		    {
			if ( fusionCondition ( o2 , o1 ) == true )
			{
			    fusion ( (AtomComponent*) dGeomGetData ( o1 ) ); //FUSION is called;
			    return true;
			}

			if ( fissionCondition ( o2 , o1 , force ) == true )
			{
			    fission ( force ); //FISSION is called
			    return true;
			}
			//if no fusion and no fission dit happen
			return false;
		    }
		
		/**********************************************************************************************/
		return false;
	    }   
	    else //there was only one or no sphere shell part of the collision
	    {
		return false; //standart collision handling by the simulations is used
	    }
	}
	else
	{
	    for ( int n = 0; n < getNumberSubcomponents (); n++ )
	    {
		if ( connection[n].subcomponent->collisionCallback ( data , o1 , o2 ) )
		    return true; // exit if collision was treated by a robot/component
	    }
	    return false; //a simpleComponent does never handle collisions itself, it uses the standard collisionCallback of the simulation
	}
    }

	bool AtomComponent::shellCollision ( dGeomID o1 , dGeomID o2 )
	{
	    if ( shell->getGeom () == o1 || shell->getGeom () == o2 )
		return true;
	    else
		return false;
	}

    void AtomComponent::doInternalStuff (const GlobalData &globalData)
    {
	
    }

    Position AtomComponent::getPosition () const
    {
	    osg::Vec3 position = getMainPrimitive ()->getPosition();
	    return Position ( position[0], position[1] , position[2] );
    }

    osg::Vec3 AtomComponent::getPositionbetweenComponents ( Component* component )
    {
	osg::Vec3 posi1 = getMainPrimitive ()->getPosition ();
	osg::Vec3 posi2 = component->getMainPrimitive ()->getPosition ();
	osg::Vec3 anchor = osg::Vec3 ( posi1[0] + ( posi2[0] - posi1[0])/2 , posi1[1] + ( posi2[1] - posi1[1])/2 , posi1[2] + ( posi2[2] - posi1[2])/2 );

	return anchor;
    }


/*    bool AtomComponent::setShell ( Primitive* newprimitive )
    {
	if ( shell != NULL )
	{
	    shell = newprimitive;
	    return true;
	}
	else
	{
	    shell = newprimitive;
	    return false;
	}
    }*/

    Primitive* AtomComponent::getMainPrimitive () const
    {
	return shell;
    }

    double AtomComponent::getMotionForce ()
    {
	double* linVel;
	linVel = (double*) dBodyGetLinearVel ( shell->getBody () );

	return sqrt ( pow ( linVel[0] , 2 ) + pow ( linVel[1] , 2 ) + pow ( linVel[2] , 2 ) )*atomconf.mass;
    }

    double AtomComponent::getCollisionForce ( AtomComponent* collAtom )
    {
	double* linVel;
	double* linVel2;

	linVel = (double*) dBodyGetLinearVel ( shell->getBody () );
	linVel2 = (double*) dBodyGetLinearVel ( collAtom->shell->getBody () );

	return sqrt ( pow ( linVel[0]*atomconf.mass - linVel2[0]*collAtom->atomconf.mass , 2 ) + pow ( linVel[1]*atomconf.mass - linVel2[1]*collAtom->atomconf.mass , 2 ) + pow ( linVel[2]*atomconf.mass - linVel2[2]*collAtom->atomconf.mass , 2 ) );
    }

    bool AtomComponent::fusionCondition ( dGeomID o1 , dGeomID o2 )
    {
	//only if there is a binding position remaining
	if ( getNumberSubcomponents () < atomconf.max_bindings )
	{
	    double force = ((AtomComponent*) dGeomGetData ( o2 ))->getCollisionForce ( (AtomComponent*) dGeomGetData ( o1 ) );
	    //only binds an AtomComponent to another, if the fusion as enough force
	    if ( force >= atomconf.binding_energy )
		if ( force < atomconf.min_fission_energy )
		{
		    return true;//FUSION is called
		}
	    return false;
	}
	else
	    return false;
    }
/**
 *now fission is allowed if there is a single connection, with a binding energy smaller than the force of the colission, but the minimum value for fission from atomconf has to be reached
 **/
    bool AtomComponent::fissionCondition ( dGeomID o1 , dGeomID o2 , double force )
    {
	for ( int n = 0; n < getNumberSubcomponents (); n++ )
	{
	    if ( force >= atomconf.min_fission_energy )
		if ( force >= ((connectionAddition*) connection[n].data)->binding_strength )
		    return true;//fission
	}
	return false;//no fission
    }

    bool AtomComponent::collisionExclusionCondition ( dGeomID o1 , dGeomID o2 )
    {
/*
	if ( ((AtomComponent*) dGeomGetData ( o1 ))->isComponentConnected ( (Component*) dGeomGetData ( o2 ) ) == true )
	{
	    return true; //in this case the collision is ignored
	}
	if ( ((AtomComponent*) dGeomGetData ( o2 ))->isComponentConnected ( (Component*) dGeomGetData ( o1 ) ) == true )
	{
	    return true; //in this case the collision is ignored
	}
*/

	if ( ((AtomComponent*) dGeomGetData ( o1 ))->hasSubcomponent ( (Component*) dGeomGetData ( o2 ) ) == true )
	{
	    return true; //in this case the collision is ignored
	}

	if ( ((AtomComponent*) dGeomGetData ( o2 ))->hasSubcomponent ( (Component*) dGeomGetData ( o1 ) ) == true )
	{
	    return true; //in this case the collision is ignored
	}

	return false;
    }



    bool AtomComponent::fusion ( AtomComponent* atom_to_fuse )
    {
	cout<<"fusion\n";

	//if the atom_to_fuse is a subcomponent of this before fusing, then the new connection only becomes a softlink
	if ( isComponentConnected ( atom_to_fuse ) == true )
	{
	    Axis axis = Axis ( ( getPosition () - atom_to_fuse->getPosition ()).toArray() );
	    HingeJoint* j1 = new HingeJoint ( getMainPrimitive () , atom_to_fuse->getMainPrimitive () , getPositionbetweenComponents ( atom_to_fuse ) , axis );
	    j1->init ( odeHandle , osgHandle , true/*false*/ , atomconf.shell_radius+atomconf.core_radius );

	    addSubcomponent ( atom_to_fuse , j1 );
	    if ( setSoftlink ( getNumberSubcomponents() - 1 , true ) != true )
		cout<<"Softlink could not be set because of wrong indexation\n";   
	    cout<<"Softlink set!\n";

	}
	//if it is a fusion with atomcomponents of other structures or free ones
	else
	{
	    //if the origins of both atoms are no leading atoms of a structure, does not exclude, that they are identical, but this should be catched by the rule above
	    if ( !( ((AtomComponent*) (atom_to_fuse->originComponent))->atomconf.leadingatom == true && ((AtomComponent*) originComponent)->atomconf.leadingatom == true ) )
	    {
		Axis axis = Axis ( ( getPosition () - atom_to_fuse->getPosition ()).toArray() );
		HingeJoint* j1 = new HingeJoint ( getMainPrimitive () , atom_to_fuse->getMainPrimitive () , getPositionbetweenComponents ( atom_to_fuse ) , axis );
		j1->init ( odeHandle , osgHandle , true/*false*/ , atomconf.shell_radius+atomconf.core_radius );

		//this is the normal atom fusion
		//switches the binding of the two components, so that the uncontrolled componments always will become subcomponents to the controlled ones
		if ( ((AtomComponent*) (atom_to_fuse->originComponent))->atomconf.leadingatom == false && ((AtomComponent*) originComponent)->atomconf.leadingatom == true )
		{
		    addSubcomponent ( atom_to_fuse , j1 );
		}
		else
		    if ( ((AtomComponent*) (atom_to_fuse->originComponent))->atomconf.leadingatom == true && ((AtomComponent*) originComponent)->atomconf.leadingatom == false )
		    {
			atom_to_fuse->addSubcomponent ( this , j1 );
		    }
	    }
	    //if two controller controlled component structures would fuse, the replication mecanism is activated
	    else
	    { 
		cout<<getNumberSubcomponents ()<<"\n";
		cout<<atom_to_fuse->getNumberSubcomponents ()<<"\n";
		replication ( atom_to_fuse );
		return true;
	    }
	}

	
	connection.back().data = new connectionAddition ();
	((connectionAddition*) connection.back().data)->binding_strength = atom_to_fuse->getCollisionForce ( this );
	return true;
    }

    bool AtomComponent::fission ( double force )
    {
	cout<<"fission\n";
	
	cout<<getNumberSubcomponents ()<<"\n";



//first creating a list of all bound subcomponents, sorting it after the binding_strength of the connections

	double binding_strength_counter = 100000000;
	int m = 0;

	while ( force > 0 && ( getNumberSubcomponents() > 0 ) )
	{
	    for ( int n = 0; n < getNumberSubcomponents (); n++ )
	    {
		if ( ((connectionAddition*) connection[n].data)->binding_strength < binding_strength_counter )
		{
		    m = n;
		    binding_strength_counter = ((connectionAddition*) connection[n].data)->binding_strength;		
		}
	    }

	    connection[m].subcomponent->resetMotorsRecursive ( ); 

	    //softlinks should not stay like they are

	    if ( removeSubcomponent ( m ) == NULL)
		cout<<"Subcomponent removal Error\n";

	    force -= binding_strength_counter;
	    binding_strength_counter = 100000000;
	}

	cout<<getNumberSubcomponents ()<<"\n";
	
	return true;
    }

    /**
     *Replication
     **/
    void AtomComponent::replication ( AtomComponent* atom_to_replicate )
    {
	cout<<"replication\n";
	

	if ( ( getNumberSubcomponentsAll () + 1 >= 4 ) && ( atom_to_replicate->getNumberSubcomponentsAll() + 1 >= 4 ) )
	{
	    Component* partA1;
	    Component* partA2;
	    Component* partA3;
	    Component* partA4;
	    Component* partB1;
	    Component* partB2;
	    Component* partB3;
	    Component* partB4;


/*************************************splitting the first structure into 4 parts********************************************/
	    partA1 = originComponent;
	    partA3 = partA1->getBestDivideComponent ( 1.0/2.0 , partA1->getNumberSubcomponentsAll () + 1 , NULL );
	    if ( partA3 != NULL )
	    {
		if ( partA3->directOriginComponent->removeSubcomponent ( partA3 ) == NULL )
		    cout<<"Subcomponent removal Error\n";
//	    partA3->removeAllSubcomponentsRecursive ();
		
//only if the soft links are between two componets, which belong to different structures now
		partA1->removeSoftlinksRecursive ();
		partA3->removeSoftlinksRecursive ();
		
		partA3->resetMotorsRecursive ( );
		
	    }
	    else
	    {
		cout<<"replication aborded because of wrong dividing\n";
		return;
	    }
	    


	    partA2 = partA1->getBestDivideComponent ( 1.0/2.0 , partA1->getNumberSubcomponentsAll () + 1 , NULL );

	    if ( partA2 != NULL )
	    {
		if ( partA2->directOriginComponent->removeSubcomponent ( partA2 ) == NULL )
		    cout<<"Subcomponent removal Error\n";
//	    partA3->removeAllSubcomponentsRecursive ();
		
//only if the soft links are between two componets, which belong to different structures now
		partA1->removeSoftlinksRecursive ();
		partA2->removeSoftlinksRecursive ();
		
		partA2->resetMotorsRecursive ( );
		
	    }
	    else
	    {
		cout<<"replication aborded because of wrong dividing\n";
		return;
	    }
	    


	    partA4 = partA3->getBestDivideComponent ( 1.0/2.0 , partA3->getNumberSubcomponentsAll () + 1 , NULL );
	    
	    if ( partA4 != NULL )
	    {
		if ( partA4->directOriginComponent->removeSubcomponent ( partA4 ) == NULL )
		    cout<<"Subcomponent removal Error\n";
//only if the soft links are between two componets, which belong to different structures now
		partA3->removeSoftlinksRecursive ();
		partA4->removeSoftlinksRecursive ();
		    
		partA4->resetMotorsRecursive ( );
	    }
	    else
	    {
		cout<<"replication aborded because of wrong dividing\n";
		return;
	    }

	

/*************************************splitting the second structure into 4 parts********************************************/

	    partB1 = atom_to_replicate->originComponent;
	    partB3 = partB1->getBestDivideComponent ( 1.0/2.0 , partB1->getNumberSubcomponentsAll () + 1 , NULL );
	    if ( partB3 != NULL )
	    {
		if ( partB3->directOriginComponent->removeSubcomponent ( partB3 ) == NULL )
		    cout<<"Subcomponent removal Error\n";
//	    partA3->removeAllSubcomponentsRecursive ();
		
//only if the soft links are between two componets, which belong to different structures now
		partB1->removeSoftlinksRecursive ();
		partB3->removeSoftlinksRecursive ();
		
		partB3->resetMotorsRecursive ( );
		
	    }
	    else
	    {
		cout<<"replication aborded because of wrong dividing\n";
		return;
	    }
	    
	    
	    partB2 = partB1->getBestDivideComponent ( 1.0/2.0 , partB1->getNumberSubcomponentsAll () + 1 , NULL );
	    
	    if ( partB2 != NULL )
	    {
		if ( partB2->directOriginComponent->removeSubcomponent ( partB2 ) == NULL )
		    cout<<"Subcomponent removal Error\n";
//	    partA3->removeAllSubcomponentsRecursive ();
		
//only if the soft links are between two componets, which belong to different structures now
		partB1->removeSoftlinksRecursive ();
		partB2->removeSoftlinksRecursive ();
		
		partB2->resetMotorsRecursive ( );
		
	    }
	    else
	    {
		cout<<"replication aborded because of wrong dividing\n";
		return;
	    }
	    


	    partB4 = partB3->getBestDivideComponent ( 1.0/2.0 , partB3->getNumberSubcomponentsAll () + 1 , NULL );
	    
	    if ( partB4 != NULL )
	    {
		if ( partB4->directOriginComponent->removeSubcomponent ( partB4 ) == NULL )
		    cout<<"Subcomponent removal Error\n";
//only if the soft links are between two componets, which belong to different structures now
		partB3->removeSoftlinksRecursive ();
		partB4->removeSoftlinksRecursive ();
		
		partB4->resetMotorsRecursive ( );
	    }
	    else
	    {
		cout<<"replication aborded because of wrong dividing\n";
		return;
	    }



//creating the four joints between the 8 new components

/*	    Axis axis = Axis ( ( partA1->getPosition () - partB2->getPosition()).toArray() );
	    HingeJoint* j1 = new HingeJoint ( partA1->getMainPrimitive () , partB2->getMainPrimitive () , partA1->getPositionbetweenComponents ( partB2 ) , axis );
	    j1->init ( odeHandle , osgHandle , true , ((AtomComponent*) partA1)->atomconf.shell_radius + ((AtomComponent*) partB2)->atomconf.core_radius );

	    axis = Axis ( ( partB1->getPosition () - partA2->getPosition()).toArray() );
	    HingeJoint* j2 = new HingeJoint ( partB1->getMainPrimitive () , partA2->getMainPrimitive () , partB1->getPositionbetweenComponents ( partA2 ) , axis );
	    j2->init ( odeHandle , osgHandle , true , ((AtomComponent*) partB1)->atomconf.shell_radius + ((AtomComponent*) partA2)->atomconf.core_radius );

	    axis = Axis ( ( partA3->getPosition () - partB3->getPosition()).toArray() );
	    HingeJoint* j3 = new HingeJoint ( partA3->getMainPrimitive () , partB3->getMainPrimitive () , partA3->getPositionbetweenComponents ( partB3 ) , axis );
	    j3->init ( odeHandle , osgHandle , true , ((AtomComponent*) partA3)->atomconf.shell_radius + ((AtomComponent*) partB3)->atomconf.core_radius );

	    axis = Axis ( ( partA4->getPosition () - partB4->getPosition()).toArray() );
	    HingeJoint* j4 = new HingeJoint ( partA4->getMainPrimitive () , partB4->getMainPrimitive () , partA4->getPositionbetweenComponents ( partB4 ) , axis );
	    j4->init ( odeHandle , osgHandle , true , ((AtomComponent*) partA4)->atomconf.shell_radius + ((AtomComponent*) partB4)->atomconf.core_radius );

	    partA1->addSubcomponent ( partB2 , j1 );
	    partB1->addSubcomponent ( partA1 , j2 );
	    partA3->addSubcomponent ( partB3 , j3 );
	    partA4->addSubcomponent ( partB4 , j4 );
*/
	}
	else
	    cout<<"No replication because only structures with four or more atoms could replicate\n";

	cout<<"end of replication\n";
    }



}

