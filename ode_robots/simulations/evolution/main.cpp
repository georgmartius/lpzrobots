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
 *   $Log$
 *   Revision 1.17  2008-05-01 22:03:54  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.16  2006/11/30 08:51:39  robot8
 *   -update of the evolution projekt
 *   -fitness changed
 *   -replication changed
 *   -added copy function
 *
 *   Revision 1.15  2006/10/20 13:52:42  robot8
 *   -update of the evolution projekt
 *   -only changed some parameter
 *
 *   Revision 1.14  2006/10/10 07:50:22  robot8
 *   -update of the evolution projekt
 *   -only changed some parameter
 *
 *   Revision 1.13  2006/09/22 08:52:20  robot8
 *   - corrected lifeCycle update
 *
 *   Revision 1.12  2006/09/22 05:27:14  robot8
 *   *** empty log message ***
 *
 *   Revision 1.11  2006/09/20 07:24:36  robot8
 *   *** empty log message ***
 *
 *   Revision 1.10  2006/09/12 09:39:25  robot8
 *   -working simulation is possible, but no fitness calculation and no selection at the moment
 *
 *   Revision 1.9  2006/09/11 12:01:31  martius
 *   *** empty log message ***
 *
 *   Revision 1.8  2006/09/08 09:16:31  robot8
 *   *** empty log message ***
 *
 *   Revision 1.7  2006/09/04 06:27:22  robot8
 *   -adding some testing key functions for manual fusion and fission
 *
 *   Revision 1.6  2006/08/31 07:34:13  robot8
 *   -temporary disabling a part oif the replication function
 *
 *   Revision 1.5  2006/08/21 11:50:59  robot8
 *   -added some commemts
 *   -update of atomcomponent
 *
 *   Revision 1.4  2006/07/25 11:29:56  robot8
 *   -test-update of atomcomponent (not working yet)
 *   -added backward reference feature for component softlinks
 *   -removed recusive destruction for fissed component-subtrees
 *
 *   Revision 1.3  2006/07/18 09:23:22  robot8
 *   -atomcomponent update
 *   -coloring simulation
 *   -one bug left: softlink removal from higher branches of the tree could not removed
 *
 *   Revision 1.1.2.4  2006/07/14 13:01:41  robot8
 *   -atomcomponent update
 *   -one bug left: softlink removal from higher branches of the tree could not removed
 *
 *   Revision 1.1.2.3  2006/07/07 07:40:16  robot8
 *   -some bugs removed to components and robots
 *   -some smaller functions implemented in component system
 *   -atomcomponents are functional, but still could not replicate
 *
 *   Revision 1.1.2.2  2006/06/16 12:07:39  robot8
 *   -component update
 *   -atomcomponents now could use fusion and fission
 *   -but still fission does not work all correctly
 *
 *   Revision 1.1.2.1  2006/06/12 12:59:08  robot8
 *   -some corrections to component system (for example now connections are only pushed as pointers to the connection vector)
 *   -created the evolution simulation for evolutionary evolution algorithms with atom like structures
 *
 *
 ***************************************************************************/

#include <ode_robots/simulation.h>

#include <ode_robots/atomodeagent.h>
#include <ode_robots/closedplayground.h>
#include <ode_robots/passivesphere.h>

#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotorspace.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

#include <ode_robots/sphererobot.h>
#include <ode_robots/schlangeservo.h>
#include <ode_robots/sphererobot3masses.h>

#include <ode_robots/atomcomponent.h>

#include <vector.h>


//physical definition part
#define REACTIONROOMWIDTH 2.5
#define REACTIONROOMLENGTH 2.5
//both density values ahve to be bigger than 1 so, that the atoms do not colide at the beginning, the higher the value, the less atoms are there
#define ATOMDENSITY 3
 //this is the space between two atoms, bigger means less robots and smaller more atoms
#define ROBOTDENSITY 6 //this is the space between two robots-atoms, bigger means less robots and smaler more robot-atoms
#define ROBOTHEIGHT 1.5
#define SHIFTBETWEENATOMSANDROBOTS_X 0.1
#define SHIFTBETWEENATOMSANDROBOTS_Y 0.1

//controller parameters


//evolution definition part
#define MAXPOPULATIONSIZE 3
#define MAXLIFECYCLE 1000

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace std;

class Evolution : public Simulation 
{

public:
    vector <Component*> components;
    vector <repSlider> replicationSlider;
    vector <AtomOdeAgent*> selectionlist;

public:

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {

    setCameraHomePos ( Pos ( 0, 0, 7), Pos ( -0.0828247 , -89.9146 , 0) );
    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.noise=0.1;
    //  global.odeConfig.setParam("gravity", 0);
    //  int chessTexture = dsRegisterTexture("chess.ppm");

    // use Playground as boundary:
    // - create pointer to playground (odeHandle contains things like world and space the 
    //   playground should be created in; odeHandle is generated in simulation.cpp)
    // - setting geometry for each wall of playground: 
    //   setGeometry(double length, double width, double	height)
    // - setting initial position of the playground: setPosition(double x, double y, double z)
    // - push playground in the global list of obstacles(globla list comes from simulation.cpp)
    Color color2 = Color ( 1 , 1 , 1 );
    color2.alpha() = 0.2;	
    OsgHandle osgHandlePlayground = osgHandle.changeColor ( color2 );
    ClosedPlayground* playground = new ClosedPlayground(odeHandle, osgHandlePlayground, osg::Vec3( REACTIONROOMWIDTH , 0.2 , REACTIONROOMLENGTH ) );
    playground->setPosition(osg::Vec3(0,0,-0.1)); // playground positionieren und generieren
    global.obstacles.push_back(playground);

    //****************

    ComponentConf cConf = Component::getDefaultConf ();
    cConf.max_force = 10;
    cConf.speed = 6;

    AtomConf aConf = AtomComponent::getDefaultAtomConf ();
    aConf.core_radius = 0.05;
    aConf.shell_radius = 0.1;
    aConf.mass = 1;
    aConf.max_bindings = 4;
    aConf.binding_energy = 0.1;
    aConf.min_fission_energy = 1.5;

    aConf.replicationSliderHandle = &replicationSlider;

//adding the controller for the component-connections

    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();

    cc.cInit=2;
    AbstractController* controller;
    DerivativeWiringConf c = DerivativeWiring::getDefaultConf ();
    DerivativeWiring* wiring;
    AtomOdeAgent* agent;

    OsgHandle osgHandle_2 = osgHandle.changeColor ( Color ( 0 , 156/255.0 , 2 ) );


    for ( double x = -REACTIONROOMWIDTH/2; x <= REACTIONROOMWIDTH/2; x += ROBOTDENSITY*(2*aConf.shell_radius) )
	for ( double y = -REACTIONROOMLENGTH/2; y <= REACTIONROOMLENGTH/2; y += ROBOTDENSITY*(2*aConf.shell_radius) +1)
	{
	    //atom
	    components.push_back ( new AtomComponent ( odeHandle , osgHandle_2 , cConf , aConf ) );
	    components.back ()->place ( Pos( x + SHIFTBETWEENATOMSANDROBOTS_X , y + SHIFTBETWEENATOMSANDROBOTS_Y , aConf.shell_radius*2 + ROBOTHEIGHT ));

	    ((AtomComponent*) components.back ())->atomconf.leadingatom = true;
	    //controller

	    controller = new InvertMotorNStep ( cc );
	    controller->setParam ("adaptrate", 0.005);
	    controller->setParam ("epsC", 0.005);
	    controller->setParam ("epsA", 0.001);
	    controller->setParam ("rootE", 3);
	    controller->setParam ("steps", 2);
	    controller->setParam ("s4avg", 5);
	    controller->setParam ("factorB",0);
//	    controller = new SineController ( 18 );
	    //wiring
	    wiring = new DerivativeWiring ( c , new ColorUniformNoise () );   
	    //agent
	    agent = new AtomOdeAgent ( plotoptions );
	    agent->init ( controller , components.back () , wiring );
	    global.agents.push_back ( agent );
	    global.configs.push_back ( controller );
	}
    
//unbound atoms

    osgHandle_2 = osgHandle.changeColor ( Color ( 2 , 156/255.0 , 0 ) );

    for ( double x = -REACTIONROOMWIDTH/2.2; x <= REACTIONROOMWIDTH/2.2; x += ATOMDENSITY*(2*aConf.shell_radius) )
	for ( double y = -REACTIONROOMLENGTH/2.2; y <= REACTIONROOMLENGTH/2.2; y += ATOMDENSITY*(2*aConf.shell_radius) )
	{
	    components.push_back ( new AtomComponent ( odeHandle , osgHandle_2 , cConf , aConf ) );
	    components.back ()->place ( Pos( x , y , 2*aConf.shell_radius ) ); 
	}

    showParams(global.configs);
    replicationSlider.empty ();


    /*    selectionlist.push_back ( (AtomOdeAgent*) globalData.agents.front () );
    globalData.configs.erase ( globalData.configs.begin () );
    delete ( globalData.agents.front() );
    globalData.agents.erase ( globalData.agents.begin () );

    selectionlist.erase ( selectionlist.begin () );*/
    

  } 

    void addCallback ( GlobalData& globalData , bool draw , bool pause )
	{




	    for ( unsigned int n = 0; n < components.size (); n++ )
		components[n]->update (); //not realy perfect, because the atoms belonging to robots are drawn an additional time by the agents   


	    //only if there are replication sliders allocated at the moment

	    
	    for ( unsigned int n = 0; n < replicationSlider.size (); n++ )
	    {		
		replicationSlider[n].slider->update (); //not realy perfect, because the atoms belonging to robots are drawn an additional time by the agents
		replicationSlider[n].bindingcounter--;

		if ( replicationSlider[n].startComponent->getDistanceToComponent ( replicationSlider[n].endComponent ) < 0.3 || replicationSlider[n].bindingcounter == 0 )
		{
		    //adding the new controllers for the two structures that do not have one up till now, that means the ones that have now leading atom
		    if ( ((AtomComponent*) replicationSlider[n].startComponent)->atomconf.leadingatom == false )
		    {
			((AtomComponent*) replicationSlider[n].startComponent)->makeComponentStructureRoot (); //to be shure that the structure stays correct when we create a new controller


	    //saving the existing controllers so hey can be used to create the new ones
	    //the two replication sliders where the startComponents atoms are leading atoms(do have agents)
	    //each pair of even and uneven controller driven atoms belong to each other (hat means belong to one replication, so that the new controllers should be build from them)
			//	    for ( unsigned int n = 0; n < replicationSlider.size (); n++ )
			//	      if ( ((AtomComponent*) replicationSlider[n].startComponent)->atomconf.leadingatom == true )
			//		;



			InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
			cc.cInit=2;
			AbstractController* controller;
			DerivativeWiringConf c = DerivativeWiring::getDefaultConf ();
			DerivativeWiring* wiring;
			AtomOdeAgent* agent;

			//controller
			controller = new InvertMotorNStep ( cc );
			controller->setParam ("adaptrate", 0.005);
			controller->setParam ("epsC", 0.005);
			controller->setParam ("epsA", 0.001);
			controller->setParam ("rootE", 3);
			controller->setParam ("steps", 2);
			controller->setParam ("s4avg", 5);
			controller->setParam ("factorB",0);
			//wiring
			wiring = new DerivativeWiring ( c , new ColorUniformNoise () );   
			//agent
			agent = new AtomOdeAgent ( plotoptions );
			agent->init ( controller ,  replicationSlider[n].startComponent , wiring );
			globalData.agents.push_back ( agent );
			globalData.configs.push_back ( controller );		

			((AtomOdeAgent*) globalData.agents.back ())->setLifeCycle ( MAXLIFECYCLE );

			((AtomComponent*) ((AtomOdeAgent*) globalData.agents.back ())->getRobot())->atomconf.leadingatom = true;

		    }
		    
		    ((AtomComponent*) replicationSlider[n].startComponent->originComponent)->enableStructureFusionRecursive ();
		    ((AtomComponent*) replicationSlider[n].endComponent->originComponent)->enableStructureFusionRecursive ();

		    ((AtomComponent*) replicationSlider[n].startComponent)->fusion ( (AtomComponent*) replicationSlider[n].endComponent );
		    ((AtomComponent::connectionAddition*) replicationSlider[n].startComponent->connection.back().data)->binding_strength = replicationSlider[n].startingdistance;

		    delete replicationSlider[n].slider;
		    replicationSlider.erase ( replicationSlider.begin () + n );
		}
	    }


	    /**********************************************************************************************/

	    //	    evolution loop
	    //      calculate fitness

	    for ( unsigned int n = 0; n < globalData.agents.size (); n++ )
	      cout<<"LifeCycle of "<<n<<" : "<<globalData.agents[n]->getRobot ()<<" = "<<((AtomOdeAgent*) globalData.agents[n])->getLifeCycle ()<<" Fitness: "<<((AtomOdeAgent*) globalData.agents[n])->getFitness ()<<"\n"; //not realy perfect, because the atoms belonging to robots are drawn an additional time by the agents


	    double fitnesscounter;
	    selectionlist.clear (); //clears all elements from the selection list, so that a new selection for this simulation step could start

	    unsigned int pushbackposition = 0;
	    vector <AtomOdeAgent*> tmpagentvec; //vector for all agents with lifecycle 0
	    vector <AtomOdeAgent*> tmpagentvec2;//vector for all agents not with life cycle 0

	    //creating a selected list from the original agent list
	    for ( unsigned int n = 0; n < globalData.agents.size (); n++ )
	    {

		if ( ((AtomOdeAgent*) globalData.agents[n])->getLifeCycle() <= 0 )

		  tmpagentvec.push_back ( (AtomOdeAgent*) globalData.agents[n] );

		else

		  tmpagentvec2.push_back ( (AtomOdeAgent*) globalData.agents[n] );
	    }

	    //tmp val for finaly approving the selection
	    bool selection_is = true;
	    
	    
	    //creating a sorted list of all agents
	    while ( tmpagentvec.size() > 0 )
	    {
		fitnesscounter = 1000000000; //not realy good, but ok for the moment

		double tmpfitness;
		for ( unsigned int m = 0; m < tmpagentvec.size (); m++ )
		{
			tmpfitness = tmpagentvec[m]->getFitness ();

			//creating new fitness selection here

			//calling the fitness function
			if ( tmpfitness < fitnesscounter )
			{
			    fitnesscounter = tmpfitness;
			    pushbackposition = m;
			}
		}

		//controlling if fitness of the selected agent is smaller than all other fitness values of agents who where notselected up till now
		for ( unsigned int a = 0; a < tmpagentvec2.size (); a++ )
		  {
		    if ( fitnesscounter > tmpagentvec2[a]->getFitness () )
		      {
			selection_is = false;
			break;
		      }
		    else
		      selection_is = true;
		  }

		if ( selection_is == true )
		    selectionlist.push_back ( tmpagentvec[pushbackposition] );

		//in all cases the agent that was tested will be removed from the tmpagentvec, so that the while loop will come to an end in all cases
		tmpagentvec.erase ( tmpagentvec.begin() + pushbackposition );
		
	    }
	    selection_is = true;

	    //	    for ( unsigned int n = 0; n < selectionlist.size (); n++ )
	    //	    cout<<selectionlist[n]<<" : "<<selectionlist[n]->getFitness ()<<"\n";

	    //after this double loop the selection list contains all agents whose lifeCycle has ended, and whose fitness values are the lowest in the whole population at the moment of calculation. It ends with the fittest idividual, the individuals less fit are at the start of the vector. That means agents are only selected if they have the lowest fitness in the population and if their lifetime is over
	    //also the life times had been updated


	    //SELECTION

	    while ( (globalData.agents.size() > MAXPOPULATIONSIZE) && (selectionlist.size () > 0) ) //only select if there are beeings whose life cycle ended
	    {	
		//deleting the controller from global configs
	      //		vector <Configurable*>::iterator eraseiterator = globalData.configs.begin ();

		    for ( unsigned int m = 0; m < globalData.configs.size (); m++ )
		    {
			if ( globalData.configs[m] == selectionlist.front()/*[n]*/->getController () )
			{
			  globalData.configs.erase ( globalData.configs.begin () + m );
			  break;
			}
		    }

		    for ( unsigned int m = 0; m < globalData.agents.size (); m++ )
		    {
		    
			if ( globalData.agents[m] == selectionlist.front() )
			{
			  
			  globalData.agents.erase ( globalData.agents.begin () + m /* eraseiterator2*/ );

			  break;
			}
		    }

		    ((AtomComponent*) selectionlist.front ()->getRobot ())->atomconf.leadingatom = false;
		    ((AtomComponent*) selectionlist.front ()->getRobot ())->removeAllSubcomponentsRecursive ();
		    ((AtomComponent*) selectionlist.front ()->getRobot ())->resetMotorsRecursive ();


		    delete ( selectionlist.front() );

		    selectionlist.erase ( selectionlist.begin () );

	    }
	    
	    //lifeCycle and fitnes update
	    for ( unsigned int n = 0; n < globalData.agents.size (); n++ )
	    {
	      ((AtomOdeAgent*) globalData.agents[n])->setLifeCycle ( ((AtomOdeAgent*) globalData.agents[n])->getLifeCycle () - 1 );
	      ((AtomOdeAgent*) globalData.agents[n])->updateFitness ();
	    }
	    

	    //all individuals who have survived the selection get a new lifeCycle of existance to aquire fitness
	    for ( unsigned int m = 0; m < globalData.agents.size (); m++ )
	    {
		if ( ((AtomOdeAgent*) globalData.agents[m])->getLifeCycle () < 0 )
		{
		    ((AtomOdeAgent*) globalData.agents[m])->setLifeCycle ( MAXLIFECYCLE );
		    ((AtomOdeAgent*) globalData.agents[m])->resetFitness ();
		}
	    }

//	    cout<<"before lifecycle update\n";

//	    cout<<"end Selection part\n";

 
	    /**********************************************************************************************/


	}


  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released

		ComponentConf compConf = Component::getDefaultConf ();
		compConf.max_force = 5;

		AtomConf atConf = AtomComponent::getDefaultAtomConf ();
		atConf.core_radius = 0.05;
		atConf.shell_radius = 0.1;
		atConf.mass = 1;
		atConf.max_bindings = 4;
		atConf.binding_energy = 0.3;
		atConf.min_fission_energy = 2;
		atConf.replicationSliderHandle = &replicationSlider;
		
		int n = 1;//heigh value for placing new atoms to teh simulation

      switch ( (char) key )
	{
	    case 'Q':
		cout<<"atom added\n";

		components.push_back ( new AtomComponent ( odeHandle , osgHandle.changeColor ( Color ( 0 , 0 , 0 ) ) , compConf , atConf ) );
		components.back ()->place ( Pos( -REACTIONROOMWIDTH/2.5 , REACTIONROOMLENGTH/2.5 , atConf.shell_radius + n )); 
		break;
	    case 'W':
		cout<<"atom added\n";
		components.push_back ( new AtomComponent ( odeHandle , osgHandle.changeColor ( Color ( 0 , 0 , 1 ) ) , compConf , atConf ) );
		components.back ()->place ( Pos( 0 , REACTIONROOMLENGTH/2.5 , atConf.shell_radius + n )); 
		break;
	    case 'E':
		cout<<"atom added\n";
		components.push_back ( new AtomComponent ( odeHandle , osgHandle.changeColor ( Color ( 0 ,  1 , 0 ) ) , compConf , atConf ) );
		components.back ()->place ( Pos( REACTIONROOMWIDTH/2.5 , REACTIONROOMLENGTH/2.5 , atConf.shell_radius + n )); 
		break;
	    case 'A':
		cout<<"atom added\n";
		components.push_back ( new AtomComponent ( odeHandle , osgHandle.changeColor ( Color ( 0 , 1 , 1 ) ) , compConf , atConf ) );
		components.back ()->place ( Pos( -REACTIONROOMWIDTH/2.5 , 0 , atConf.shell_radius + n )); 
		break;
	    case 'S':
		cout<<"atom added\n";
		components.push_back ( new AtomComponent ( odeHandle , osgHandle.changeColor ( Color ( 1 , 0 , 0 ) ) , compConf , atConf ) );
		components.back ()->place ( Pos( 0 , 0 , atConf.shell_radius + n )); 
		break;
	    case 'D':
		cout<<"atom added\n";
		components.push_back ( new AtomComponent ( odeHandle , osgHandle.changeColor ( Color ( 1 , 0 , 1 ) ) , compConf , atConf ) );
		components.back ()->place ( Pos( REACTIONROOMWIDTH/2.5 , 0 , atConf.shell_radius + n )); 
		break;
	    case 'Y':
		cout<<"atom added\n";
		components.push_back ( new AtomComponent ( odeHandle , osgHandle.changeColor ( Color ( 1 , 1 , 0 ) ) , compConf , atConf ) );
		components.back ()->place ( Pos( -REACTIONROOMWIDTH/2.5 , -REACTIONROOMLENGTH/2.5 , atConf.shell_radius + n )); 
		break;
	    case 'X':
		cout<<"atom added\n";
		components.push_back ( new AtomComponent ( odeHandle , osgHandle.changeColor ( Color ( 1 , 1 , 1 ) ) , compConf , atConf ) );
		components.back ()->place ( Pos( 0 , -REACTIONROOMLENGTH/2.5 , atConf.shell_radius + n )); 
		break;
	    case 'C':
		cout<<"atom added\n";
		components.push_back ( new AtomComponent ( odeHandle , osgHandle.changeColor ( Color ( 0.5 , 0.5 , 0.5 ) ) , compConf , atConf ) );
		components.back ()->place ( Pos( REACTIONROOMWIDTH/2.5 , -REACTIONROOMLENGTH/2.5 , atConf.shell_radius + n )); 
		break;
	    case 'R':
		cout<<"KEY REPLICATION\n";
		cout<<"replicationSlider-Adress: "<<&replicationSlider<<"\n";
		((AtomComponent*) globalData.agents[0]->getRobot())->replication ( ((AtomComponent*) globalData.agents[1]->getRobot()) );
		//((AtomComponent*)components[0])->replication ( (AtomComponent*) components[1] );

		cout<<replicationSlider.size ()<<"\n";

		cout<<"END OF KEY REPLICATION\n";
		break;

	    case 'M':
	      cout<<"CREATE COPIES OF ALL INDIVIDUALS\n";
	      for ( unsigned int n = 0; n < globalData.agents.size (); n++ )
		((AtomComponent*) globalData.agents[n]->getRobot())->copyCompleteStructure ( Pos ( 0 , 0 , 10 ) , NULL );
	      break;

	    case 'T':
		cout<<"TERMINATE ALL FREE ATOMS\n";

		for ( unsigned int n = 0; n < components.size (); n++)
		{
		    if ( components[n]->connection.size() == 0 && components[n]->backwardreference.size () == 0 && components[n]->directOriginComponent == components[n] && ((AtomComponent*) components[n])->atomconf.leadingatom == false )
		    {
		      delete ( components[n] );
			components.erase ( components.begin() + n );
		    }
		}
		break;


	    case 'I':
		cout<<"PRINTING INFORMATION\n";
		for ( unsigned int n = 0; n < globalData.agents.size (); n++ )
		{
		    cout<<"################################################Robot##"<<n<<"################################################\n";
		    printComponentInfo ( (Component*) globalData.agents[n]->getRobot() );
		}
		break;
		/*	    case 'L':
		    cout<<"#####################################Complete Component List#############################################\n";
		for ( unsigned int n = 0; n < components.size (); n++ )
		{
		    cout<<"*******************Component**"<<components[n]<<"******************\n";   
		    cout<<"origin component: "<<((AtomComponent*) components[n] )->originComponent<<"\n";
		    cout<<"direct origin component: "<<((AtomComponent*) components[n] )->directOriginComponent<<"\n";
		    cout<<"leading atom: "<<((AtomComponent*) components[n] )->atomconf.leadingatom<<"\n";
		    cout<<"number of Subcomponents: "<<((AtomComponent*) components[n] )->getNumberSubcomponents ()<<"\n";
		    cout<<"number of all Subcomponents: "<<((AtomComponent*) components[n])->getNumberSubcomponentsAll ()<<"\n";
		    
		    for ( unsigned int m = 0; m < ((AtomComponent*) components[n])->connection.size (); m++ )
		    {
			cout<<"------------Connection-"<<m<<"------------\n";
			cout<<"adress: "<<&((AtomComponent*) components[n])->connection[m]<<"\n";
			cout<<"softlink: "<<((AtomComponent*) components[n])->connection[m].softlink<<"\n";
			cout<<"data: "<<((AtomComponent*) components[n])->connection[m].data<<"\n";
			cout<<"subcomponent: "<<((AtomComponent*) components[n])->connection[m].subcomponent<<"\n";
		    }

		    for ( unsigned int m = 0; m < (components[n])->backwardreference.size (); m++ )
		    {
			cout<<"------------Backwardreference-"<<m<<"------------\n";
			cout<<"component: "<<components[n]->backwardreference[m]<<"\n";
			
		    }

		}
		break;
		*/
/*	    case 'M' :
		cout<<"#####################################connection vector capacity analysis#############################################\n";
		for ( unsigned int n = 0; n < components.size(); n++)
		    cout<<"Component "<<n<<" holds "<<components[n]->connection.size ()<<" and can hold "<<components[n]->connection.capacity ()<<" connections\n";
		break;
*/
		
		//external fission/fusion test
		/*	    case 'V': 
		((AtomComponent*) components[0])->fusion ( (AtomComponent*) components[1] );
//		cout<<*((double*) ((AtomComponent*) components[0])->connection[0].data)<<"\n";
		break;
	    case 'B': 
		((AtomComponent*) components[1])->fusion ( (AtomComponent*) components[2] );
//		cout<<*((double*) ((AtomComponent*) components[1])->connection[0].data)<<"\n";
		break;
	    case 'N': 
		((AtomComponent*) components[2])->fusion ( (AtomComponent*) components[3] );
//		cout<<*((double*) ((AtomComponent*) components[2])->connection[0].data)<<"\n";
		break;
	    case 'M': 
		((AtomComponent*) components[0])->fusion ( (AtomComponent*) components[2] );
//		cout<<*((double*) ((AtomComponent*) components[0])->connection[1].data)<<"\n";
		break;
	    case ';': 
		cout<<"backward binding strength: \n";
		for ( unsigned int n = 0; n < ((AtomComponent*) components[0])->backwardreference.size(); n++ )
		    cout<<*((double*) ((AtomComponent*) components[0])->backwardreference[n]->getConnection ( (AtomComponent*) components[0] )->data)<<"\n";
		cout<<"normal connection-binding strength: \n";
		for ( unsigned int n = 0; n < ((AtomComponent*) components[0])->connection.size(); n++ )
		    cout<<*((double*) ((AtomComponent*) components[0])->connection[n].data)<<"\n";
*/
		((AtomComponent*) components[0])->fission ( 1000 );
		break;
	    case ':': 
/*		cout<<"directOrigin connection-binding-strength\n";
		cout<<*((double*) ((AtomComponent*) components[1])->directOriginComponent->getConnection ( (AtomComponent*) components[1] )->data)<<"\n";
		cout<<"backward binding strength: \n";
		for ( unsigned int n = 0; n < ((AtomComponent*) components[1])->backwardreference.size(); n++ )
		    cout<<*((double*) ((AtomComponent*) components[1])->backwardreference[n]->getConnection ( (AtomComponent*) components[1] )->data)<<"\n";
		cout<<"normal connection-binding strength: \n";
		for ( unsigned int n = 0; n < ((AtomComponent*) components[1])->connection.size(); n++ )
		    cout<<*((double*) ((AtomComponent*) components[1])->connection[n].data)<<"\n";
*/
		((AtomComponent*) components[1])->fission ( 1000 );
		break;
	    case '_': 
/*		cout<<"directOrigin connection-binding-strength\n";
		cout<<*((double*) ((AtomComponent*) components[2])->directOriginComponent->getConnection ( (AtomComponent*) components[2] )->data)<<"\n";
		cout<<"backward binding strength: \n";
		for ( unsigned int n = 0; n < ((AtomComponent*) components[0])->backwardreference.size(); n++ )
		    cout<<*((double*) ((AtomComponent*) components[0])->backwardreference[n]->getConnection ( (AtomComponent*) components[0] )->data)<<"\n";
		cout<<"normal connection-binding strength: \n";
		for ( unsigned int n = 0; n < ((AtomComponent*) components[0])->connection.size(); n++ )
		    cout<<*((double*) ((AtomComponent*) components[0])->connection[n].data)<<"\n";
*/
		((AtomComponent*) components[2])->fission ( 1000 );
		break;
	case 'L':
	  InvertMotorNStepConf cc;
	  cc = InvertMotorNStep::getDefaultConf();
	  cc.cInit=2;
	  AbstractController* controller;
	  DerivativeWiringConf c;
	  c = DerivativeWiring::getDefaultConf ();
	  DerivativeWiring* wiring;
	  AtomOdeAgent* agent;
	  
	  //controller
	  controller = new InvertMotorNStep ( cc );
	  controller->setParam ("adaptrate", 0.005);
	  controller->setParam ("epsC", 0.005);
	  controller->setParam ("epsA", 0.001);
	  controller->setParam ("rootE", 3);
	  controller->setParam ("steps", 2);
	  controller->setParam ("s4avg", 5);
	  controller->setParam ("factorB",0);
	  //wiring
	  wiring = new DerivativeWiring ( c , new ColorUniformNoise () );   
	  //agent
	  agent = new AtomOdeAgent ( plotoptions );
	  agent->init ( controller ,  components.back () , wiring );
	  globalData.agents.push_back ( agent );
	  globalData.configs.push_back ( controller );		
	  
	  ((AtomOdeAgent*) globalData.agents.back ())->setLifeCycle ( MAXLIFECYCLE );
	  
	  ((AtomComponent*) ((AtomOdeAgent*) globalData.agents.back ())->getRobot())->atomconf.leadingatom = true;
	  break;

	case 'K':
	  if ( globalData.agents.size () > 0 )
	    {
	      delete ( globalData.agents.front () );
	      globalData.agents.erase ( globalData.agents.begin () );
	      
	      break;
	    }



	    default:
		return false;
		break;
		

	}
    }
    return false;
  }

    void printComponentInfo ( Component* comp )
	{
	    cout<<"*******************Component**"<<comp<<"******************\n";   
	    cout<<"origin component: "<<((AtomComponent*) comp )->originComponent<<"\n";
	    cout<<"direct origin component: "<<((AtomComponent*) comp )->directOriginComponent<<"\n";
	    cout<<"leading atom: "<<((AtomComponent*) comp )->atomconf.leadingatom<<"\n";
	    cout<<"number of Subcomponents: "<<((AtomComponent*) comp)->getNumberSubcomponents ()<<"\n";
	    cout<<"number of all Subcomponents: "<<((AtomComponent*) comp)->getNumberSubcomponentsAll ()<<"\n";
	    
	    for ( unsigned int m = 0; m < ((AtomComponent*) comp)->connection.size (); m++ )
	    {
		cout<<"------------Connection-"<<m<<"------------\n";
		cout<<"adress: "<<&((AtomComponent*) comp)->connection[m]<<"\n";
		cout<<"softlink: "<<((AtomComponent*) comp)->connection[m].softlink<<"\n";
		cout<<"data: "<<((AtomComponent*) comp)->connection[m].data<<"\n";
		cout<<"subcomponent: "<<((AtomComponent*) comp)->connection[m].subcomponent<<"\n";


	    }

	    for ( unsigned int m = 0; m < ((AtomComponent*) comp)->backwardreference.size (); m++ )
	    {
		cout<<"------------Backwardreference-"<<m<<"------------\n";
		cout<<"component: "<<comp->backwardreference[m]<<"\n";

	    }

//	    cout<<"*********************************************************\n";   
	    
	    for ( unsigned int m = 0; m < ((AtomComponent*) comp)->connection.size (); m++ )
	    {
		if ( !comp->connection[m].softlink )
		    printComponentInfo ( comp->connection[m].subcomponent );

	    }

	}
  

};



int main (int argc, char **argv)
{ 
  Evolution sim;
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}
