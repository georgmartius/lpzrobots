/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
 *    mai00bvz@studserv.uni-leipzig.de                                     *
 *    joergweide84@aol.com (robot12)                                       *
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
 *   Revision 1.5  2009-07-15 12:56:25  robot12
 *   the simulation
 *
 *   Revision 1.4  2009/07/06 15:06:35  robot12
 *   bugfix
 *
 *   Revision 1.3  2009/07/02 15:24:53  robot12
 *   update and add new class InvertedFitnessStrategy
 *
 *   Revision 1.2  2009/07/02 10:13:36  guettler
 *   updated copyright section
 *
 *   Revision 1.1  2009/07/02 10:10:38  guettler
 *   template for genetic algorithms using the lpzrobots library ga_tools with cycled simulations
 *
 *   Revision 1.2  2009/07/02 10:05:59  guettler
 *   added example erasing an agent after one cycle and creating new ones
 *
 *   Revision 1.1  2009/04/23 14:17:34  guettler
 *   new: simulation cycles, first simple implementation, use the additional method bool restart() for starting new cycles, template simulation can be found in template_cycledSimulation (originally taken from template_onerobot)
 *
 *
 ***************************************************************************/
#include <stdio.h>
#include <vector>

// include ode library
#include <ode/ode.h>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>

// include simulation environment stuff
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>

// used wiring
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>
#include <selforg/invertmotornstep.h>


// used robot
#include <ode_robots/nimm2.h>
#include <ode_robots/nimm4.h>


// used arena
#include <ode_robots/playground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>

// used controller
#include <selforg/invertmotorspace.h>

// used ga_tools
#include <ga_tools/SingletonGenAlgAPI.h>
#include <ga_tools/Generation.h>
#include <ga_tools/Individual.h>
#include <ga_tools/Gen.h>
#include <ga_tools/TemplateValue.h>

#include "TemplateCycledGaSimulationFitnessStrategy.h"

#include <selforg/trackablemeasure.h>
#include <selforg/statistictools.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

PlotOption opt1(GuiLogger);						// a plot Option for the generation measure to guilogger
PlotOption opt2(File);							// a plot Option for the generation measure to file
PlotOption optGen(File);						// a plot Option for gen measure to file


class ThisSim : public Simulation {
public:
	ThisSim(int numInd=25) : Simulation(), numberIndividuals(numInd) {}

	virtual ~ThisSim() {
		SingletonGenAlgAPI::destroyAPI();
	}


  // starting function (executed once at the beginning of the simulation loop/first cycle)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
	// ga_tool initing
	// first we need some variables.
	RandGen random;									// a random generator
	IFitnessStrategy* invertedFitnessStr;			// the inverted fitness strategy
	IGenerationSizeStrategy* gSStr;					// a generation size strategy (fix -> by 100 Individual -> look at run)
	ISelectStrategy* selStr;						// a select strategy
	IMutationFactorStrategy* mutFaStr;				// a mutation factor strategy for the mutation strategy (standard)
	IMutationStrategy* mutStr;						// a mutation strategy (will be standard)
	IRandomStrategy* randomStr;						// a random strategy for the gens
	GenPrototype* pro1;								// the 4 prototypes for the gens 2 Sensors - 2 Engines ==> 4 neuron connections
	GenPrototype* pro2;
	GenPrototype* pro3;
	GenPrototype* pro4;

	// next we need the general strategys for the alg.
	gSStr = SingletonGenAlgAPI::getInstance()->createFixGenerationSizeStrategy(numberIndividuals);
	SingletonGenAlgAPI::getInstance()->setGenerationSizeStrategy(gSStr);
	selStr = SingletonGenAlgAPI::getInstance()->createTournamentSelectStrategy(&random);
	SingletonGenAlgAPI::getInstance()->setSelectStrategy(selStr);

	// after this we need the fitness strategy
	fitnessStr = new TemplateCycledGaSimulationFitnessStrategy();
	invertedFitnessStr = SingletonGenAlgAPI::getInstance()->createInvertedFitnessStrategy(fitnessStr);
	SingletonGenAlgAPI::getInstance()->setFitnessStrategy(invertedFitnessStr);

	// now its time to creat all needed for the gens.
	mutFaStr = SingletonGenAlgAPI::getInstance()->createStandartMutationFactorStrategy();
	mutStr = SingletonGenAlgAPI::getInstance()->createValueMutationStrategy(mutFaStr,50);
	randomStr = SingletonGenAlgAPI::getInstance()->createDoubleRandomStrategy(&random,-2.0,4.0,0.0);
	pro1 = SingletonGenAlgAPI::getInstance()->createPrototype("P1",randomStr,mutStr);
	pro2 = SingletonGenAlgAPI::getInstance()->createPrototype("P2",randomStr,mutStr);
	pro3 = SingletonGenAlgAPI::getInstance()->createPrototype("P3",randomStr,mutStr);
	pro4 = SingletonGenAlgAPI::getInstance()->createPrototype("P4",randomStr,mutStr);
	SingletonGenAlgAPI::getInstance()->insertGenPrototype(pro1);
	SingletonGenAlgAPI::getInstance()->insertGenPrototype(pro2);
	SingletonGenAlgAPI::getInstance()->insertGenPrototype(pro3);
	SingletonGenAlgAPI::getInstance()->insertGenPrototype(pro4);

	// at last we create all measure
	opt1.setName("opt1");
	//opt2.setName("opt2");
	SingletonGenAlgAPI::getInstance()->enableMeasure(opt1);
	SingletonGenAlgAPI::getInstance()->enableMeasure(opt2);
	//SingletonGenAlgAPI::getInstance()->getPlotOptionEngine()->addPlotOption(opt1);
	//SingletonGenAlgAPI::getInstance()->getPlotOptionEngine()->addPlotOption(opt2);
	optGen.setName("optGen");
	SingletonGenAlgAPI::getInstance()->enableGenContextMeasure(optGen);
	//SingletonGenAlgAPI::getInstance()->getPlotOptionEngineForGenContext()->addPlotOption(optGen);

	// prepare the first generation
	SingletonGenAlgAPI::getInstance()->prepare(numberIndividuals,numberIndividuals/3,false);

	// so we are ready to start the alg! Need is only the simulation!
	// also we must create the robots and agents for the simulation
	createBots(global);

	// first: position(x,y,z) second: view(alpha,beta,gamma)
	// gamma=0;
	// alpha == horizontal angle
	// beta == vertical angle
	setCameraHomePos(Pos(37.3816, 23.0469, 200.818),  Pos(0.0404358, -86.7151, 0));
	// initialization
	// - set noise to 0.1
	global.odeConfig.noise=0.05;
	// set realtimefactor to maximum
    global.odeConfig.setParam("realtimefactor",0);

    global.odeConfig.setParam("cameraspeed",100000);
    // use Playground as boundary:
	// - create pointer to playground (odeHandle contains things like world and space the
	//   playground should be created in; odeHandle is generated in simulation.cpp)
	// - setting geometry for each wall of playground:
	//   setGeometry(double length, double width, double	height)
	// - setting initial position of the playground: setPosition(double x, double y, double z)
	// - push playground in the global list of obstacles(globla list comes from simulation.cpp)
	//
	// and add passive spheres as obstacles
	// - create pointer to sphere (with odehandle, osghandle and
	//   optional parameters radius and mass,where the latter is not used here) )
	// - set Pose(Position) of sphere
	// - set a texture for the sphere


  }

  /**
   * restart() is called at the second and all following starts of the cycle
   * The end of a cycle is determined by (simulation_time_reached==true)
   * @param the odeHandle
   * @param the osgHandle
   * @param globalData
   * @return if the simulation should be restarted; this is false by default
   */
  virtual bool restart(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
	  // we want 10 runs!
	  if(this->currentCycle==11) {
		  FOREACH(std::vector<TrackableMeasure*>,storageMeasure,i) {
			  printf("%s hat folgende Entropy: %lf\n",(*i)->getName().c_str(),(*i)->getValue());
		  }

		  SingletonGenAlgAPI::getInstance()->update();

		  // after 100 runs, we stop and make all clean
		  // clean GA TOOLS
		  SingletonGenAlgAPI::getInstance()->getPlotOptionEngine()->removePlotOption(GuiLogger);
		  SingletonGenAlgAPI::getInstance()->getPlotOptionEngine()->removePlotOption(File);
		  SingletonGenAlgAPI::getInstance()->getPlotOptionEngineForGenContext()->removePlotOption(File);

		  SingletonGenAlgAPI::destroyAPI();

		  //clean robots
		  while(global.agents.size()>0) {
			  OdeAgent* agent = *global.agents.begin();
			  AbstractController* controller = agent->getController();
			  OdeRobot* robot = agent->getRobot();
			  AbstractWiring* wiring = agent->getWiring();

			  global.configs.erase(std::find(global.configs.begin(),global.configs.end(),controller));
			  delete controller;

			  delete robot;
			  delete wiring;

			  delete (agent);
			  global.agents.erase(global.agents.begin());
		  }

		  while(global.obstacles.size()>0) {
			  std::vector<AbstractObstacle*>::iterator iter = global.obstacles.begin();
			  delete (*iter);
			  global.obstacles.erase(iter);
		  }

		  //clean measure
		  entropyMeasure.clear();
		  while(storageMeasure.size()>0) {
			  std::vector<TrackableMeasure*>::iterator iter = storageMeasure.begin();
			  delete (*iter);
			  storageMeasure.erase(iter);
		  }

		  return false; //stop running
	  }

	  //clean actual entropy measure list
	  entropyMeasure.clear();

	  // step in the alg.
	  SingletonGenAlgAPI::getInstance()->update();
	  RandGen random;											// a random generator
	  SingletonGenAlgAPI::getInstance()->select();
	  SingletonGenAlgAPI::getInstance()->crosover(&random);
	  // now we must delete all robots and agents from the simulation and create new robots and agents
	  // can be optimized by check, which individual are killed --> only this robots killing!
	  while(global.agents.size()>0) {
		  OdeAgent* agent = *global.agents.begin();
		  AbstractController* controller = agent->getController();
		  OdeRobot* robot = agent->getRobot();
		  AbstractWiring* wiring = agent->getWiring();

		  global.configs.erase(std::find(global.configs.begin(),global.configs.end(),controller));
		  delete controller;

		  delete robot;
		  delete wiring;

		  delete (agent);
		  global.agents.erase(global.agents.begin());
	  }

	  while(global.obstacles.size()>0) {
		  std::vector<AbstractObstacle*>::iterator iter = global.obstacles.begin();
		  delete (*iter);
		  global.obstacles.erase(iter);
	  }

	 createBots(global);

    // restart!
    return true;
  }


  /** optional additional callback function which is called every simulation step.
      Called between physical simulation step and drawing.
      @param draw indicates that objects are drawn in this timestep
      @param pause always false (only called of simulation is running)
      @param control indicates that robots have been controlled this timestep
   */
  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    // if simulation_time_reached is set to true, the simulation cycle is finished
    if (this->sim_step%100==0)
    {
    	std::cout << "time: " << this->sim_step/100 << "s" << std::endl;
    }
    if (this->sim_step>=6000)
    {
      simulation_time_reached=true;
    }

    //make a step in the measure
    FOREACH(std::vector<TrackableMeasure*>,entropyMeasure,i) {
    	(*i)->step();
    }
  }

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
	{
	default:
	  return false;
	  break;
	}
    }
    return false;
  }

private:
	//std::vector<OdeAgent*>
	void createBots(GlobalData& global) {
		OdeRobot* vehicle;
		OdeAgent* agent;
		Playground* playground;

		for(int ind=0;ind<numberIndividuals;ind++) {
			Individual* individual = SingletonGenAlgAPI::getInstance()->getEngine()->getActualGeneration()->getIndividual(ind);

			playground = new Playground(odeHandle, osgHandle, osg::Vec3(18, 0.2, 0.5));
			playground->setPosition(osg::Vec3((double)(ind%5)*19.0,19.0*(double)(ind/5),0.05)); // playground positionieren und generieren
			// register playground in obstacles list
			global.obstacles.push_back(playground);

			// use Nimm2 vehicle as robot:
			// - get default configuration for nimm2
			// - activate bumpers, cigar mode and infrared front sensors of the nimm2 robot
			// - create pointer to nimm2 (with odeHandle, osg Handle and configuration)
			// - place robot
			Nimm2Conf c = Nimm2::getDefaultConf();
			c.force   = 4;
			c.bumper  = true;
			c.cigarMode  = true;
			// c.irFront = true;
			vehicle = new Nimm2(odeHandle, osgHandle, c, ("Nimm2"+individual->getName()).c_str());
			vehicle->place(Pos((double)(ind%5)*19.0,19.0*(double)(ind/5),0.0));

			// read the gen values and create the neuron matrix
			matrix::Matrix init(2,2);
			double v1,v2,v3,v4;
			TemplateValue<double>* value = dynamic_cast<TemplateValue<double>*>(individual->getGen(0)->getValue());
			value!=0?v1=value->getValue():v1=0.0;
			value = dynamic_cast<TemplateValue<double>*>(individual->getGen(1)->getValue());
			value!=0?v2=value->getValue():v2=0.0;
			value = dynamic_cast<TemplateValue<double>*>(individual->getGen(2)->getValue());
			value!=0?v3=value->getValue():v3=0.0;
			value = dynamic_cast<TemplateValue<double>*>(individual->getGen(3)->getValue());
			value!=0?v4=value->getValue():v4=0.0;
			init.val(0,0) = v1;
			init.val(0,1) = v2;
			init.val(1,0) = v3;
			init.val(1,1) = v4;

			// create pointer to controller
			// push controller in global list of configurables
			InvertMotorNStep *controller = new InvertMotorNStep(init);
			global.configs.push_back(controller);

			// create pointer to one2onewiring
			One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));

			// create pointer to agent
			// initialize pointer with controller, robot and wiring
			// push agent in globel list of agents
			agent = new OdeAgent(plotoptions);
			agent->init(controller, vehicle, wiring);
			global.agents.push_back(agent);

			//showParams(global.configs);

			// create measure for the agent
			// and connect the measure with the fitness strategy
			std::list<Trackable*> trackableList;
			trackableList.push_back(vehicle);
			TrackableMeasure* trackableEntropy = new TrackableMeasure(trackableList,("E Nimm2 of "+individual->getName()).c_str(),ENTSLOW,playground->getCornerPointsXY(),X | Y, 18);
			//StatisticTools* statsOfAgent = new StatisticTools();
			//statsOfAgent->addMeasure(trackableEntropy);
			//agent->addInspectable(statsOfAgent);
			fitnessStr->m_storage.push_back(&trackableEntropy->getValueAddress());
			entropyMeasure.push_back(trackableEntropy);
			storageMeasure.push_back(trackableEntropy);
		}
	}

	TemplateCycledGaSimulationFitnessStrategy* fitnessStr;		// the fitness strategy
	int numberIndividuals;										// number of individuals
	std::vector<TrackableMeasure*> entropyMeasure;				// all active measures for the entropy.
	std::vector<TrackableMeasure*> storageMeasure;				// all measures for the entropy.
};


int main (int argc, char **argv)
{
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;

}

