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
 *   Revision 1.10  2006-09-07 05:50:17  robot8
 *   -corrected recursion error in atomcomponent::update
 *
 *   Revision 1.9  2006/09/04 06:28:03  robot8
 *   -adding some testing key functions for manual fusion and fission
 *
 *   Revision 1.8  2006/08/31 07:33:56  robot8
 *   -temporary disabling a part oif the replication function
 *
 *   Revision 1.7  2006/08/21 11:50:45  robot8
 *   -added some commemts
 *   -update of atomcomponent
 *
 *   Revision 1.6  2006/08/02 09:26:54  martius
 *   using namespace std
 *
 *   Revision 1.5  2006/08/01 11:01:05  robot8
 *   -working makeComponentStructureRoot and getStrongtestSoftlinkofStructure functions in atomcomponent
 *   -not yet working correct completly
 *
 *   Revision 1.3  2006/07/18 09:23:22  robot8
 *   -atomcomponent update
 *   -coloring simulation
 *   -one bug left: softlink removal from higher branches of the tree could not removed
 *
 *   Revision 1.1.2.6  2006/07/11 07:51:14  robot3
 *   cvslog added
 *
 *                                                                         *
 ***************************************************************************/

#define TESTBOOLVAL true

#include "atomcomponent.h"

using namespace std;

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
	    for ( unsigned int n = 0; n < connection.size (); n++ )
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
	    for ( unsigned int n = 0; n < connection.size (); n++ )
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

	for ( unsigned int n = 0; n < connection.size (); n++ )
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

	for ( unsigned int n = 0; n < connection.size (); n++ )
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
	for ( unsigned int n = 0; n < connection.size (); n++ )
	{
	    connection[n].joint->update ();
	    if ( connection[n].softlink == false )
		connection[n].subcomponent->update ();
	}
    }

    void AtomComponent::place ( const Pos &pos )
    {
	Position newpos;

 	shell->setPosition ( osg::Vec3 ( ((Pos)pos).toPosition().x , ((Pos)pos).toPosition().y , ((Pos)pos).toPosition().z ) );

	for ( unsigned int n = 0; n < connection.size (); n++ )
	{
	    newpos = connection[n].subcomponent->getPosition () - ((Pos)pos).toPosition ();
	    if ( connection[n].softlink == false )
		connection[n].subcomponent->place ( *(new Pos ( newpos )) );
	}
    }

    void AtomComponent::place (const osg::Matrix &pose)
    {
	
    }

    bool AtomComponent::collisionCallback (void *data, dGeomID o1, dGeomID o2)
    {
//	cout<<"x\n";
	//is the shell of this atomcomponent is involved in the collision?
	if ( shellCollision ( o1 , o2 ) == true )
	{
//	    cout<<"shellCollision = true\n";
	    //does it colide with another ode geom from type sphere?
	    if ( dGeomGetClass ( o1 ) == dSphereClass && dGeomGetClass ( o2 ) == dSphereClass ) //only if two atoms colide, all other cases are handled by the default collision handling of the simulations
	    {
//		cout<<"dSphereClass  = true\n";
		if ( collisionExclusionCondition ( o1 , o2 ) == true )
		{
		    return true; //in this case the collision is ignored
		}
		
		/***************************Fusion***or***Fission*********************************************/
		/*First the decission if there is fusion or fission*/
//		cout<<"Collision with: "<<((AtomComponent*) dGeomGetData ( o2 ))->getCollisionForce ( (AtomComponent*) dGeomGetData ( o1 ) )<<" force\n";
		
		double force = ((AtomComponent*) dGeomGetData ( o2 ))->getCollisionForce ( (AtomComponent*) dGeomGetData ( o1 ) );
//		cout<<"deciding which o is which one\n";
		if ( shell->getGeom () == o1 )
		{
		    if ( fusionCondition ( o1 , o2 ) == true )
		    {
			 fusion ( (AtomComponent*) dGeomGetData ( o2 ) ); //FUSION is called;
			 return true;
		    }
		    else
		    if ( fissionCondition ( o1 , o2 , force ) == true )
		    {
			cout<<"fission\n";
			cout<<"target of fission: "<<(AtomComponent*) dGeomGetData ( o1 )<<" atom causing fission: "<<(AtomComponent*) dGeomGetData ( o2 )<<"\n";
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
			else
			    if ( fissionCondition ( o2 , o1 , force ) == true )
			    {
				cout<<"fission\n";
				cout<<"target of fission: "<<(AtomComponent*) dGeomGetData ( o2 )<<" atom causing fission: "<<(AtomComponent*) dGeomGetData ( o1 )<<"\n";
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
//    	    cout<<"atom with possible error: "<<this<<"\n";
	    for ( unsigned int n = 0; n < connection.size (); n++ )
	    {
//		cout<<"after loop start\n";
		//collisions should not ne treated over softlinks
		if ( connection[n].softlink == false )
		    if ( connection[n].subcomponent->collisionCallback ( data , o1 , o2 ) )
			return true; // exit if collision was treated by a subcomponent
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

    AtomComponent::componentConnection* AtomComponent::getStrongestSoftlinkofStructure ()
    {
	double tmp_binding_strength = 0;

	componentConnection* tmpconnection = NULL;
	for ( unsigned int n = 0; n < connection.size (); n++ )
	{
	    if ( connection[n].softlink == false ) //recursion only is used if the connection is not a  softlink
	    {
		tmpconnection = ((AtomComponent*) connection[n].subcomponent)->getStrongestSoftlinkofStructure ();

		if ( tmpconnection != NULL ) // only if there was a connection that is a possible softlink candidate , so that there is no wrong mem access
		    if ( ((connectionAddition*) tmpconnection->data)->binding_strength > tmp_binding_strength )
		    {
			tmp_binding_strength = ((connectionAddition*) tmpconnection->data)->binding_strength;
		    }
	    }
	}

	//get best outgoing softlink
	for ( unsigned int n = 0; n < connection.size (); n++ )
	    if ( connection[n].softlink == true)
		if ( ((connectionAddition*) connection[n].data)->binding_strength > tmp_binding_strength )
		{
		    //it is important that the two components are not connected, because softlinks within the removed substructure are not important
		    if ( !connection[n].subcomponent->isComponentConnected ( this ) )
		    {
			tmp_binding_strength = ((connectionAddition*) connection[n].data)->binding_strength;
			tmpconnection = &connection[n];
		    }
		}
	//get best incomming softlink
	for ( unsigned int n = 0; n < backwardreference.size (); n++ )
	    for ( unsigned int m = 0; m < backwardreference[n]->connection.size(); m++ )
	    {
		if ( backwardreference[n]->connection[m].subcomponent == this )
		    if ( ((connectionAddition*) backwardreference[n]->connection[m].data)->binding_strength > tmp_binding_strength )
		    {
			//it is important that the two components are not connected, because softlinks within the removed substructure are not important
			if ( !backwardreference[n]->isComponentConnected ( this ) )
			{
			    tmp_binding_strength = ((connectionAddition*) backwardreference[n]->connection[m].data)->binding_strength;
			    tmpconnection = &(backwardreference[n]->connection[m]);
			}
		}
	    }

	return tmpconnection;

    }

    void AtomComponent::makeComponentStructureRoot()
    {

	if ( directOriginComponent != this ) //it only works, and is necessary if the new root of the structure isn't a root already (which it should be if it refers itself as its origin)
	{
	    if ( directOriginComponent != /*directOriginComponent->directOriginComponent*/originComponent )
	    {
//		if ( directOriginComponent->getConnection ( this )->softlink == false )
		    ( (AtomComponent*) directOriginComponent)->makeComponentStructureRoot ();
	    }
	    
	    Component* tmp_directOriginComponent = directOriginComponent;

	    void* tmp_connectionAddition = new connectionAddition ();
//	    cout<<"connection adress: "<<directOriginComponent->getConnection ( this )<<"\n";
//	    cout<<"connection data adress: "<<directOriginComponent->getConnection ( this )->data<<"\n";

	    ((connectionAddition*) tmp_connectionAddition)->binding_strength = ((connectionAddition*) ( directOriginComponent->getConnection ( this ) )->data)->binding_strength;


	    directOriginComponent->removeSubcomponent ( this );

	    Axis axis = Axis ( ( getPosition () - tmp_directOriginComponent->getPosition ()).toArray() );
	    HingeJoint* newjoint = new HingeJoint ( getMainPrimitive () , tmp_directOriginComponent->getMainPrimitive () , getPositionbetweenComponents ( tmp_directOriginComponent ) , axis );
	    newjoint->init ( odeHandle , osgHandle , TESTBOOLVAL/*truealse*/ , atomconf.shell_radius+atomconf.core_radius );

	   
	    addSubcomponent ( tmp_directOriginComponent , newjoint , false );


	    //adding the data Pointer to
	    (connection.back().data) = tmp_connectionAddition;


	    connection.back ().subcomponent->updateOriginsRecursive ( this );
	}
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
	if ( connection.size () < (unsigned int) atomconf.max_bindings )
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
	for ( unsigned int n = 0; n < connection.size (); n++ )
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

	if ( ( (Component*) dGeomGetData ( o1 ))->hasSubcomponent ( (Component*) dGeomGetData ( o2 ) ) == true )
	{
	    return true; //in this case the collision is ignored
	}

	if ( ( (Component*) dGeomGetData ( o2 ))->hasSubcomponent ( (Component*) dGeomGetData ( o1 ) ) == true )
	{
	    return true; //in this case the collision is ignored
	}

	return false;
    }


    bool AtomComponent::fusion ( AtomComponent* atom_to_fuse )
    {
	if ( !( atomconf.fusionDisabled == true || atom_to_fuse->atomconf.fusionDisabled == true) )
	{
	    cout<<"fusion\n";
	    
	    //if the atom_to_fuse is a subcomponent of this before fusing, then the new connection only becomes a softlink
	    if ( isComponentConnected ( atom_to_fuse ) == true )
	    {
		cout<<"fusion case 1\n";
		cout<<"atom to bind on: "<<this<<" atom to fuse: "<<atom_to_fuse<<"\n";
		osgHandle.color.alpha () = 0.3;
		
		Axis axis = Axis ( ( getPosition () - atom_to_fuse->getPosition ()).toArray() );
		HingeJoint* j1 = new HingeJoint ( getMainPrimitive () , atom_to_fuse->getMainPrimitive () , getPositionbetweenComponents ( atom_to_fuse ) , axis );
		j1->init ( odeHandle , osgHandle , TESTBOOLVAL , atomconf.shell_radius+atomconf.core_radius );
		
		osgHandle.color.alpha () = 1;
		
		
		//a subcomponent is added as a softlink
		addSubcomponent ( atom_to_fuse , j1 , true );
		
		
		void* testp = new connectionAddition ();
		connection.back().data = testp;
		((connectionAddition*) connection.back().data)->binding_strength = atom_to_fuse->getCollisionForce ( this );

		cout<<"end of fusion 1\n";
		return true;
		
	    }
	    //if it is a fusion with atomcomponents of other structures or free ones
	    else
	    {
		cout<<"fusion case 2\n";
		//if the origins of both atoms are no leading atoms of a structure, does not exclude, that they are identical, but this should be catched by the rule above
		if ( !( ((AtomComponent*) (atom_to_fuse->originComponent))->atomconf.leadingatom == true && ((AtomComponent*) originComponent)->atomconf.leadingatom == true ) )
		{
		    //this is the normal atom fusion
		    //switches the binding of the two components, so that the uncontrolled componments always will become subcomponents to the controlled ones
		    if (((AtomComponent*) (atom_to_fuse->originComponent))->atomconf.leadingatom == false && ((AtomComponent*) originComponent)->atomconf.leadingatom == true )
		    {
			cout<<"fusion case 2_\n";
			Axis axis = Axis ( ( getPosition () - atom_to_fuse->getPosition ()).toArray() );
			HingeJoint* j1 = new HingeJoint ( getMainPrimitive () , atom_to_fuse->getMainPrimitive () , getPositionbetweenComponents ( atom_to_fuse ) , axis );
			j1->init ( odeHandle , osgHandle , TESTBOOLVAL/*truefalse*/ , atomconf.shell_radius+atomconf.core_radius );
			
			cout<<"fusion case 2a\n";
			cout<<"atom to bind on: "<<this<<" atom to fuse: "<<atom_to_fuse<<"\n";
			//before binding the structure has to be changed, so that the tree structure stays consitent
			atom_to_fuse->makeComponentStructureRoot ();

			addSubcomponent ( atom_to_fuse , j1 , false );
			
			void* testp = new connectionAddition ();
			connection.back().data = testp;
			((connectionAddition*) connection.back().data)->binding_strength = atom_to_fuse->getCollisionForce ( this );
			cout<<"end of fusion 2a\n";
			return true;
			
		    }
		    else
		    {
			
			if ( ((AtomComponent*) (atom_to_fuse->originComponent))->atomconf.leadingatom == true && ((AtomComponent*) originComponent)->atomconf.leadingatom == false )
			{
			    Axis axis = Axis ( ( getPosition () - atom_to_fuse->getPosition ()).toArray() );
			    HingeJoint* j1 = new HingeJoint ( getMainPrimitive () , atom_to_fuse->getMainPrimitive () , getPositionbetweenComponents ( atom_to_fuse ) , axis );
			    j1->init ( odeHandle , osgHandle , TESTBOOLVAL/*truefalse*/ , atomconf.shell_radius+atomconf.core_radius );
			    
			    cout<<"fusion case 2b\n";
			    cout<<"atom to bind on: "<<atom_to_fuse<<" atom to fuse: "<<this<<"\n";
			    this->makeComponentStructureRoot ();
			    atom_to_fuse->addSubcomponent ( this , j1 , false );
			    
			    void* testp = new connectionAddition ();
			    connection.back().data = testp;
			    ((connectionAddition*) connection.back().data)->binding_strength = this->getCollisionForce ( atom_to_fuse );
			    cout<<"end of fusion 2b\n";
			    return true;
			}
			
			//the case false false is not handled
		    }
		}
		//if two controller controlled component structures would fuse, the replication mecanism is activated
		else
		{
		    cout<<"fusion case 3\n";
		    //cout<<getNumberSubcomponents ()<<"\n";
		    //cout<<atom_to_fuse->getNumberSubcomponents ()<<"\n";
		    //replication ( atom_to_fuse );
		    return false/*true*/;
		}
	    }
	    return false;
	}
	return false;
    }

void AtomComponent::disableStructureFusionRecursive ()
{
    atomconf.fusionDisabled = true;
    for ( unsigned int n = 0; n < connection.size ();n++ )
    {
	((AtomComponent*) connection[n].subcomponent)->disableStructureFusionRecursive ();
    }
}

void AtomComponent::enableStructureFusionRecursive ()
{
    atomconf.fusionDisabled = false;
    for ( unsigned int n = 0; n < connection.size ();n++ )
    {
	((AtomComponent*) connection[n].subcomponent)->enableStructureFusionRecursive ();
    }
}


bool AtomComponent::fission ( double force )
    {
	cout<<"fission\n";
	
//first creating a list of all bound subcomponents, sorting it after the binding_strength of the connections

	double binding_strength_counter = 100000000;
	int m = 0;
	AtomComponent* tmpremovedsub = NULL;

	while ( force > 0 && ( ( connection.size() > 0 ) || ( backwardreference.size () > 0 )  || ( directOriginComponent != this) ) )
	{
	    //searches for the weakes connection
	    for ( unsigned int n = 0; n < connection.size(); n++ )
	    {
		if ( ((connectionAddition*) connection[n].data)->binding_strength < binding_strength_counter )
		{
		    m = n + 1; //zero is used for the removing of the direct origin
		    binding_strength_counter = ((connectionAddition*) connection[n].data)->binding_strength;		
		    cout<<"connection selected\n";
		}
	    }

	    //looks if the removing of the origin connection is possible
	    if ( directOriginComponent != this ) //only if there is a connection to an directOrigin, and the component is not the root of the structure itself
	    {

		if ( ((connectionAddition*) directOriginComponent->getConnection ( this )->data)->binding_strength < binding_strength_counter )
		{
		    m = 0;
		    binding_strength_counter = ((connectionAddition*) directOriginComponent->getConnection ( this )->data)->binding_strength;
		    cout<<"direct origin selected\n";
		}
	    }


	    for ( unsigned int n = 0; n < backwardreference.size (); n++ )
	    {
		if ( ( (connectionAddition*) backwardreference[n]->getConnection ( this )->data )->binding_strength < binding_strength_counter )
		{
		    m = (-1*n)-1; //using the negativ numbers to symolise that it is a backward reference that was selected as the connection to remove; -1 because 0 could be used only for one case, later there is a calculation of +1 to compensate the -1 from this line
		    binding_strength_counter = ((connectionAddition*) backwardreference[n]->getConnection ( this )->data )->binding_strength;
		    cout<<"backwardreference selected\n";
		}
	    }

	    cout<<"m : "<<m<<"\n";

	//now removing one of the connection possibilities
	    //removing a normal connection
	    if ( m > 0 )
	    {
		m = m - 1;

		tmpremovedsub = (AtomComponent*) removeSubcomponent ( m );
		    
	    }
	    else
	    {
		//removing the origin connection
		if ( m == 0 )
		{
		    tmpremovedsub = (AtomComponent*) directOriginComponent->removeSubcomponent ( this );
		}
		//removing a backward reference connection
		else
		{
		    m = -(m + 1);//changing the number to positive values again
		    tmpremovedsub = (AtomComponent*) backwardreference[m]/*->getConnection ( this )->subcomponent*/->removeSubcomponent ( this );

		}
	    }
	    cout<<"end of removal section\n";



//consequences of removing a not-softlink-connection

	    if ( tmpremovedsub == NULL)
		    cout<<"Subcomponent removal Error\n";
		else
		    //removing was successfull
		{
//		    cout<<tmpremovedsub<<"\n";
//		    cout<<tmpremovedsub->connection.size()<<"\n";

		    componentConnection* tmp_softlinkconnection = ((AtomComponent*) tmpremovedsub)->getStrongestSoftlinkofStructure ();
		    cout<<"found tmp_softlinkconnection: "<<tmp_softlinkconnection<<"\n";

//		    cout<<"Softlink adress:"<<tmp_softlinkconnection<<"\n";

		    //setting the motor values to zero, but only if there was no softlink found and the substructure will be seperated from the rest of the structure
		    if ( tmp_softlinkconnection == NULL )	
		    {
			cout<<"no softlink connection\n";
			tmpremovedsub->resetMotorsRecursive ( ); 

		    }
		    else //if there is a softlink incomming or outgoing in or from the removed substructure
		    // if ( tmp_softlinkconnection != NULL ) 
			for ( unsigned int n = 0;  n < tmp_softlinkconnection->subcomponent->backwardreference.size (); n++ )
			{
			    cout<<"n: "<<n<<"\n";
			    for ( unsigned int m = 0; m < tmp_softlinkconnection->subcomponent->backwardreference[n]->connection.size() ; m++ )
			    {
				cout<<m<<"\n";
				if ( tmp_softlinkconnection->subcomponent->backwardreference[n]->getConnection ( m ) == tmp_softlinkconnection )
				{
				    cout<<m<<"\n";
				    //if true, it is an incoming softlink to the structure belonging to tmpremovedsub
				    if ( tmpremovedsub->isComponentConnected ( tmp_softlinkconnection->subcomponent ) )
				    {
					Component* tmpcomp1;
					Component* tmpcomp2;
					tmpcomp1 = tmp_softlinkconnection->subcomponent->backwardreference[n];
					tmpcomp2 = tmp_softlinkconnection->subcomponent;

					tmpcomp1->removeSubcomponent ( tmpcomp2 );					
					
					((AtomComponent*) tmpcomp2)->makeComponentStructureRoot (); //creating a new structure root					
					cout<<"end of first option\n";
					

					Axis axis = Axis ( ( tmpcomp1->getPosition () - tmpcomp2->getPosition ()).toArray() );
					HingeJoint* newjoint = new HingeJoint ( tmpcomp1->getMainPrimitive () , tmpcomp2->getMainPrimitive () , tmpcomp1->getPositionbetweenComponents ( tmpcomp2 ) , axis );
					newjoint->init (  odeHandle ,  osgHandle , TESTBOOLVAL/*truefalse*/ , ((AtomComponent*) tmpcomp1)->atomconf.shell_radius + ((AtomComponent*) tmpcomp1)->atomconf.core_radius );
					tmpcomp1->addSubcomponent ( tmpcomp2 , newjoint , false );

					//adding the binding energy
					void* testp = new connectionAddition ();
					tmpcomp1->connection.back().data = testp;
					((connectionAddition*) tmpcomp1->connection.back().data)->binding_strength = ((AtomComponent*) tmpcomp2)->getCollisionForce ( (AtomComponent*) tmpcomp1 );
					tmp_softlinkconnection = NULL;
					break;
				    }
				    else
					//if false it is an outgoing softlink to the structure belonging to tmpremovedsub
					if ( tmpremovedsub->isComponentConnected ( tmp_softlinkconnection->subcomponent->backwardreference[n] ) )
					{
 					    Component* tmpcomp1;
					    Component* tmpcomp2;
					    tmpcomp1 = tmp_softlinkconnection->subcomponent->backwardreference[n];
					    tmpcomp2 = tmp_softlinkconnection->subcomponent;
					    

					    tmpcomp1->removeSubcomponent ( tmpcomp2  ); //deleting the softlink,origin update is no problem, because the connection was a softlink
					    cout<<"end of sencond option\n";
					    ((AtomComponent*) tmpcomp1)->makeComponentStructureRoot ();
					    //is backwardref deleted??
					    
					    //creating new joint
					    
					    Axis axis = Axis ( ( tmpcomp2->getPosition () - tmpcomp1->getPosition ()).toArray() );
					    HingeJoint* newjoint = new HingeJoint ( tmpcomp2->getMainPrimitive () , tmpcomp1->getMainPrimitive () , tmpcomp2->getPositionbetweenComponents ( tmpcomp1 ) , axis );
					    newjoint->init ( odeHandle ,  osgHandle , TESTBOOLVAL/*truefalse*/ , ((AtomComponent*) tmpcomp2)->atomconf.shell_radius + ((AtomComponent*) tmpcomp2)->atomconf.core_radius );
					    
					    tmpcomp2->addSubcomponent ( tmpcomp1 , newjoint , false );

					    //adding the binding energy
					    void* testp = new connectionAddition ();
					    tmpcomp2->connection.back().data = testp;
					    ((connectionAddition*) tmpcomp2->connection.back().data)->binding_strength = ((AtomComponent*) tmpcomp1)->getCollisionForce ( (AtomComponent*) tmpcomp2 );

//					    tmpcomp1->updateOriginsRecursive ( tmpcomp2 );

					    tmp_softlinkconnection = NULL;
					    break;
					}
				}

			    }
			    if ( tmp_softlinkconnection == NULL )
				break;
			}
		}

	    force -= binding_strength_counter;
	    binding_strength_counter = 100000000;
	}
		    cout<<"end of fission\n";
	return true;
    }

    /**
     *Replication
     **/
void AtomComponent::replication ( AtomComponent* atom_to_replicate /*, vector <repSlider>* replicationSlider */)
    {
	vector <repSlider>* replicationSlider;
	cout<<"replication\n";
	

	if ( ( originComponent->getNumberSubcomponentsAll () + 1 >= 4 ) && ( atom_to_replicate->originComponent->getNumberSubcomponentsAll() + 1 >= 4 ) )
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

	    cout<<"\n start splitting for first structure \n";
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
		cout<<"replication aborded because of wrong dividing for A3\n";
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
		cout<<"replication aborded because of wrong dividing for A2\n";
		return;
	    }
	    


	    //special threatment fpr the last splitting, because of the possibility that B3 is only a one-atom-struture
	    if ( ( partA3->getNumberSubcomponentsAll () + 1 ) > 1 )
		partA4 = partA3->getBestDivideComponent ( 1.0/2.0 , partA3->getNumberSubcomponentsAll () + 1 , NULL );
	    else //in this case B1 is used because there had to be the rest atoms up to the minimum 4 atoms
		partA4 = partA1->getBestDivideComponent ( 1.0/2.0 , partA1->getNumberSubcomponentsAll () + 1 , NULL );

	    
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
		cout<<"replication aborded because of wrong dividing for A4\n";
		return;
	    }

		    cout<<"\n end splitting for first structure \n";

/*************************************splitting the second structure into 4 parts********************************************/


	    cout<<"\n start splitting for second structure \n";

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
		cout<<"replication aborded because of wrong dividing for B3\n";
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
		cout<<"replication aborded because of wrong dividing for B2\n";
		return;
	    }
	    

	    //special threatment for the last splitting, because of the possibility that B3 is only a one-atom-struture
	    if ( ( partB3->getNumberSubcomponentsAll () + 1 ) > 1 )
		partB4 = partB3->getBestDivideComponent ( 1.0/2.0 , partB3->getNumberSubcomponentsAll () + 1 , NULL );
	    else //in this case B1 is used because there had to be the rest atoms up to the minimum 4 atoms
		partB4 = partB1->getBestDivideComponent ( 1.0/2.0 , partB1->getNumberSubcomponentsAll () + 1 , NULL );
	    
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
		cout<<"replication aborded because of wrong dividing for B4\n";
		return;
	    }

	    cout<<"\n end splitting for second structure \n";
	    cout<<partA1<<"|"<<partA2<<"|"<<partA3<<"|"<<partA4<<"|"<<partB1<<"|"<<partB2<<"|"<<partB3<<"|"<<partB4<<"\n";

//creating the four joints between the 8 new components

	    Axis axis = Axis ( ( partA1->getPosition () - partB2->getPosition()).toArray() );
	    SliderJoint* j1 = new SliderJoint ( partA1->getMainPrimitive () , partB2->getMainPrimitive () , partA1->getPositionbetweenComponents ( partB2 ) , axis );
	    j1->init ( odeHandle , osgHandle , /*true*/TESTBOOLVAL , ((AtomComponent*) partA1)->atomconf.shell_radius + ((AtomComponent*) partB2)->atomconf.core_radius );
	    cout<<"1\n";

	    axis = Axis ( ( partB1->getPosition () - partA2->getPosition()).toArray() );
	    SliderJoint* j2 = new SliderJoint ( partB1->getMainPrimitive () , partA2->getMainPrimitive () , partB1->getPositionbetweenComponents ( partA2 ) , axis );
	    j2->init ( odeHandle , osgHandle , /*true*/TESTBOOLVAL , ((AtomComponent*) partB1)->atomconf.shell_radius + ((AtomComponent*) partA2)->atomconf.core_radius );
	    cout<<"2\n";

	    axis = Axis ( ( partA3->getPosition () - partB3->getPosition()).toArray() );
	    SliderJoint* j3 = new SliderJoint ( partA3->getMainPrimitive () , partB3->getMainPrimitive () , partA3->getPositionbetweenComponents ( partB3 ) , axis );
	    j3->init ( odeHandle , osgHandle , /*true*/TESTBOOLVAL , ((AtomComponent*) partA3)->atomconf.shell_radius + ((AtomComponent*) partB3)->atomconf.core_radius );
	    cout<<"3\n";

	    axis = Axis ( ( partA4->getPosition () - partB4->getPosition()).toArray() );
	    SliderJoint* j4 = new SliderJoint ( partA4->getMainPrimitive () , partB4->getMainPrimitive () , partA4->getPositionbetweenComponents ( partB4 ) , axis );
	    j4->init ( odeHandle , osgHandle , /*true*/TESTBOOLVAL , ((AtomComponent*) partA4)->atomconf.shell_radius + ((AtomComponent*) partB4)->atomconf.core_radius );

//	    j1->setParam ( dParamLoStop , -dInfinity );
//	    j1->setParam ( dParamHiStop , dInfinity );
	    cout<<"before VelParam-Setting\n";
	    j1->setParam ( dParamVel , -0.5 );
	    j1->setParam ( dParamFMax , 30 );
	    j2->setParam ( dParamVel , -0.5 );
	    j2->setParam ( dParamFMax , 30 );
	    j3->setParam ( dParamVel , -0.5 );
	    j3->setParam ( dParamFMax , 30 );
	    j4->setParam ( dParamVel , -0.5 );
	    j4->setParam ( dParamFMax , 30 );


	    repSlider rps1 , rps2 , rps3 , rps4;
	    rps1.startComponent = partA1; rps1.endComponent = partB2; rps1.slider = j1; rps1.startingdistance = rps1.startComponent->getDistanceToComponent ( rps1.endComponent );
	    rps1.bindingcounter = 100;
	    replicationSlider->push_back ( rps1 );

	    rps2.startComponent = partB1; rps2.endComponent = partA2; rps2.slider = j2; rps2.startingdistance = rps2.startComponent->getDistanceToComponent ( rps2.endComponent );
	    rps2.bindingcounter = 100;
	    replicationSlider->push_back ( rps2 );

	    rps3.startComponent = partA3; rps3.endComponent = partB3; rps3.slider = j3; rps3.startingdistance = rps3.startComponent->getDistanceToComponent ( rps3.endComponent );
	    rps3.bindingcounter = 100;
	    replicationSlider->push_back ( rps3 );

	    rps4.startComponent = partA4; rps4.endComponent = partB4; rps4.slider = j4; rps4.startingdistance = rps4.startComponent->getDistanceToComponent ( rps4.endComponent );
	    rps4.bindingcounter = 100;
	    replicationSlider->push_back ( rps4 );

	    //disabling the ability to fuse of all structures
		((AtomComponent*) partA1->originComponent)->disableStructureFusionRecursive ();
		((AtomComponent*) partB1->originComponent)->disableStructureFusionRecursive ();
		((AtomComponent*) partA3->originComponent)->disableStructureFusionRecursive ();
		((AtomComponent*) partA4->originComponent)->disableStructureFusionRecursive ();


	}
	else
	    cout<<"No replication because only structures with four or more atoms could replicate\n";

	cout<<"end of replication\n";
    }



}


