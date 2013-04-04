/***************************************************************************
 *   Copyright (C) 2008-2011 LpzRobots development team                    *
 *    Joerg Weider   <joergweide84 at aol dot com> (robot12)               *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *    Joern Hoffmann <jhoffmann at informatik dot uni-leipzig dot de       *
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
#include <stdio.h>
#include <vector>

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
// only for deleting
#include <ga_tools/ValueMutationStrategy.h>
#include <ga_tools/StandartMutationFactorStrategy.h>
#include <ga_tools/DoubleRandomStrategy.h>

#include "TemplateCycledGaSimulationFitnessStrategy.h"

#include <selforg/trackablemeasure.h>
#include <selforg/statistictools.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

//this 3 PlotOptions are needed for some measures. They will bring us some data on the screen and save all to a log file.
PlotOption opt1(GuiLogger); // a plot Option for the generation measure to guilogger
PlotOption opt2(File); // a plot Option for the generation measure to file
PlotOption optGen(File); // a plot Option for gene measure to file

/**
 * This class is our simulation. It simulate the robots in there playground
 */
class ThisSim : public Simulation
{
public:
  /**
   * constructor
   * creates the simulation and define how much robots are inside. Over this it define the number of
   * individuals inside the genetic algorithm
   * @param numInd (int) number of individuals and robots
   */
  ThisSim(int numInd = 25) :
    Simulation(), numberIndividuals(numInd)
  {
  }

  /**
   * destructor
   * make sure, that the gen. algorithm will be cleared
   */
  virtual
  ~ThisSim()
  {
    SingletonGenAlgAPI::destroyAPI();
  }

  /** starting function (executed once at the beginning of the simulation loop/first cycle)
   * this must create our gen. alg.
   * @param odeHandle
   * @param osgHandle
   * @param global
   */
  void
  start(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
      GlobalData& global)
  {
    // ga_tool initialising
    // First we need some variables.
    RandGen random; // a random generator
    IFitnessStrategy* invertedFitnessStr;       // the inverted fitness strategy
    IGenerationSizeStrategy* gSStr;             // a generation size strategy
    ISelectStrategy* selStr;                    // a select strategy
    GenPrototype* pro1;                         // the 4 prototypes for the genes 2 Sensors - 2 Engines ==> 4 neuron connections
    GenPrototype* pro2;
    GenPrototype* pro3;
    GenPrototype* pro4;

    // Next we need the general strategies for the algorithm.
    // - a GenerationSizeStrategy: Here we take a fixed size strategy. This means every generation has the size of "numberIndividuals"
    // - a SelectStrategy: Here we take a tournament strategy which tests 2 individuals. The better one will win.
    gSStr = SingletonGenAlgAPI::getInstance()->createFixGenerationSizeStrategy((int)((numberIndividuals - (numberIndividuals / 10)) / 2));
    SingletonGenAlgAPI::getInstance()->setGenerationSizeStrategy(gSStr);
    selStr = SingletonGenAlgAPI::getInstance()->createTournamentSelectStrategy(&random);
    SingletonGenAlgAPI::getInstance()->setSelectStrategy(selStr);

    // After this we need the fitness strategy.
    // Here we need our own strategy! But our strategy will be higher if the individual are better.
    // So we need a inverted fitness strategy because the genetic algorithm will optimise again zero.
    // More details on this strategies can be found in the belonging header files.
    fitnessStr = new TemplateCycledGaSimulationFitnessStrategy();
    invertedFitnessStr
        = SingletonGenAlgAPI::getInstance()->createInvertedFitnessStrategy(fitnessStr);
    SingletonGenAlgAPI::getInstance()->setFitnessStrategy(invertedFitnessStr);

    // Now its time to create all needed stuff for the genes.
    // - mutation strategy for the prototypes
    // - random strategy for the prototypes
    // - and the 4 prototypes for the genes:
    mutFaStr
        = SingletonGenAlgAPI::getInstance()->createStandartMutationFactorStrategy();
    // The second value means the mutation probability in 1/1000. Normal is a value lower than max. 5%.
    // But we have so few individuals, that we need a higher mutation (33,3%).
    mutStr = SingletonGenAlgAPI::getInstance()->createValueMutationStrategy(
        mutFaStr, 50);
    // The last parameters ensure that the created genes lay inside the interval from -2 to +2.
    randomStr = SingletonGenAlgAPI::getInstance()->createDoubleRandomStrategy(&random, -2.0, 4.0, 0.0);
    // The prototypes need a name, a random strategy to create random genes and a mutation strategy to mutate existing genes.
    pro1 = SingletonGenAlgAPI::getInstance()->createPrototype("P1", randomStr, mutStr);
    pro2 = SingletonGenAlgAPI::getInstance()->createPrototype("P2", randomStr, mutStr);
    pro3 = SingletonGenAlgAPI::getInstance()->createPrototype("P3", randomStr, mutStr);
    pro4 = SingletonGenAlgAPI::getInstance()->createPrototype("P4", randomStr, mutStr);
    SingletonGenAlgAPI::getInstance()->insertGenPrototype(pro1);
    SingletonGenAlgAPI::getInstance()->insertGenPrototype(pro2);
    SingletonGenAlgAPI::getInstance()->insertGenPrototype(pro3);
    SingletonGenAlgAPI::getInstance()->insertGenPrototype(pro4);

    // At last we create all interesting measures (PlotOptions).
    opt1.setName("opt1");
    opt2.setName("opt2");
    SingletonGenAlgAPI::getInstance()->enableMeasure(opt1);
    SingletonGenAlgAPI::getInstance()->enableMeasure(opt2);
    optGen.setName("optGen");
    SingletonGenAlgAPI::getInstance()->enableGenContextMeasure(optGen);

    // Prepare the first generation:
    // We can use "run" for a automatically run or we must control all ourself like here!
    // So we must prepare the first generation, for this the algorithm must know how many individuals he should create,
    // how much will die on the end and if he should make an automatically update of the statistic values.
    // The automatically update isn't possible because before we need a run of the simulation, so we make it later ourself (param false)!
    SingletonGenAlgAPI::getInstance()->prepare((int)((numberIndividuals - (numberIndividuals / 10)) / 2),
                    numberIndividuals - (((int)((numberIndividuals - (numberIndividuals / 10)) / 2)) * 2),
                    &random,false);

    // So we are now ready to start the algorithm!
    // But without the simulation we have no fun with the algorithm. ;) The only we just need is the simulation!
    // Also we must create the robots and agents for the simulation:
    createBots(global);

    // First: position(x,y,z) second: view(alpha,beta,gamma)
    // gamma=0;
    // alpha == horizontal angle
    // beta == vertical angle
    setCameraHomePos(Pos(37.3816, 23.0469, 200.818), Pos(0., -90.0, 0.));
    // TODO: disable camera tracking (static (CameraManipulator) instead of CameraManipulatorTV)

    // initialisation
    // - set noise to 0.05
    global.odeConfig.noise = 0.05;
    // set realtimefactor to maximum
    global.odeConfig.setParam("realtimefactor", 0);

  }

  /**
   * restart() is called at the second and all following starts of the cycle
   * The end of a cycle is determined by (simulation_time_reached==true)
   * @param the odeHandle
   * @param the osgHandle
   * @param globalData
   * @return if the simulation should be restarted;
   */
  virtual bool
  restart(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    // We would like to have 10 runs!
    // after it we must clean all and return false because we don't want a new restart
    // TODO: use abort criterion provided by ga_tools (not implemented yet?)
    if (this->currentCycle == 11)
    {
      // print all entropies which we have measured
      FOREACH(std::vector<TrackableMeasure*>,storageMeasure,i)
      {
        printf("%s has following entropy: %lf\n", (*i)->getName().c_str(), (*i)->getValue());
      }

      // update the genetic algorithm statistical data and make a step in the measure
      SingletonGenAlgAPI::getInstance()->update();
      SingletonGenAlgAPI::getInstance()->measureStep(currentCycle + 1);

      // after 10 runs we stop and make all clean
      // clean GA TOOLS
      SingletonGenAlgAPI::getInstance()->getPlotOptionEngine()->removePlotOption(GuiLogger);
      SingletonGenAlgAPI::getInstance()->getPlotOptionEngine()->removePlotOption(File);
      SingletonGenAlgAPI::getInstance()->getPlotOptionEngineForGenContext()->removePlotOption(File);

      // the parameter true means, that the API should delete the most strategies inside.
      SingletonGenAlgAPI::destroyAPI(true);

      // after this we must delete some strategies by use self
      // TODO: the API should delete the following strategies.
      delete mutStr;
      delete mutFaStr;
      delete randomStr;

      //delete the own fitness strategy
      delete fitnessStr;

      //clean robots
      while (global.agents.size() > 0)
      {
        OdeAgent* agent = *global.agents.begin();
        AbstractController* controller = agent->getController();
        OdeRobot* robot = agent->getRobot();
        AbstractWiring* wiring = agent->getWiring();

        global.configs.erase(std::find(global.configs.begin(),
            global.configs.end(), controller));
        delete controller;

        delete robot;
        delete wiring;

        delete (agent);
        global.agents.erase(global.agents.begin());
      }

      // clean the playgrounds
      while (global.obstacles.size() > 0)
      {
        std::vector<AbstractObstacle*>::iterator iter =
            global.obstacles.begin();
        delete (*iter);
        global.obstacles.erase(iter);
      }

      //clean measures
      entropyMeasure.clear();
      while (storageMeasure.size() > 0)
      {
        std::vector<TrackableMeasure*>::iterator iter = storageMeasure.begin();
        delete (*iter);
        storageMeasure.erase(iter);
      }

      return false; //stop running
    }

    RandGen random; // a random generator

    // clean actual entropy measure list
    entropyMeasure.clear();

    // Step in the algorithm:
    // - update the statistical values inside the genetic algorithm
    // - make a step in the measure
    // - select the individual which will be killed by use of their statistical values.
    // - and generate new individuals
    SingletonGenAlgAPI::getInstance()->update();
    SingletonGenAlgAPI::getInstance()->measureStep(currentCycle + 1);
    SingletonGenAlgAPI::getInstance()->select();
    SingletonGenAlgAPI::getInstance()->crossover(&random);

    // Now we must delete all robots and agents from the simulation and create new robots and agents.
    // TODO: This can be optimised by a check which individuals are killed --> kill only this robots!
    while (global.agents.size() > 0)
    {
      OdeAgent* agent = *global.agents.begin();
      AbstractController* controller = agent->getController();
      OdeRobot* robot = agent->getRobot();
      AbstractWiring* wiring = agent->getWiring();

      global.configs.erase(std::find(global.configs.begin(),
          global.configs.end(), controller));
      delete controller;

      delete robot;
      delete wiring;

      delete (agent);
      global.agents.erase(global.agents.begin());
    }

    // Delete all playgrounds.
    // The other way is to find the right playground, but this need more time than to delete the old and make some new!
    while (global.obstacles.size() > 0)
    {
      std::vector<AbstractObstacle*>::iterator iter = global.obstacles.begin();
      delete (*iter);
      global.obstacles.erase(iter);
    }

    // Create the Bots and Agents for the next simulation.
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
  virtual void
  addCallback(GlobalData& globalData, bool draw, bool pause, bool control)
  {
    // if 100 steps over, 1s is over
    if (globalData.sim_step % 100 == 0)
    {
      std::cout << "time: " << globalData.sim_step / 100 << "s" << std::endl;
    }

    // if simulation_time_reached is set to true, the simulation cycle is finished
    // here we finish after one minute (and a restart follows immediately)
    if (globalData.sim_step >= 6000)
    {
      simulation_time_reached = true;
    }

    // make a step in the entropy measure
    FOREACH(std::vector<TrackableMeasure*>,entropyMeasure,i)
    {
      (*i)->step();
    }
  }

  // add own key handling stuff here, just insert some case values
  virtual bool
  command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key,
      bool down)
  {
    if (down)
    { // only when key is pressed, not when released
      switch ((char) key)
      {
        default:
          return false;
          break;
      }
    }
    return false;
  }

private:
  /**
   * This function creates the robots and agents for one simulation.
   * @param global
   */
  void
  createBots(GlobalData& global)
  {
    OdeRobot* vehicle; // the robot
    OdeAgent* agent; // the agent
    Playground* playground; // the playground for the created robot

    for (int ind = 0; ind < numberIndividuals; ind++)
    {
      // At fist we need the individuals from the genetic algorithm because their genes
      // say us which values are inside the neuron matrix of the robot!
      Individual* individual = SingletonGenAlgAPI::getInstance()->getEngine()->getActualGeneration()->getIndividual(ind);

      // Next we need a playground for the robot:
      // Use Playground as boundary:
      // - create pointer to playground (odeHandle contains things like world and space the
      //   playground should be created in; odeHandle is generated in simulation.cpp)
      // - setting geometry for each wall of playground:
      //   setGeometry(double length, double width, double        height)
      // - setting initial position of the playground: setPosition(double x, double y, double z)
      // - push playground in the global list of obstacles(global list comes from simulation.cpp)
      playground = new Playground(odeHandle, osgHandle, osg::Vec3(18, 0.2, 0.5));
      playground->setPosition(osg::Vec3((double) (ind % (int) sqrt(numberIndividuals)) * 19.0,
          19.0 * (double) (ind / (int) sqrt(numberIndividuals)), 0.05)); // position and generate playground
      // register playground in obstacles list
      global.obstacles.push_back(playground);

      // Use Nimm2 vehicle as robot:
      // - get default configuration for nimm2
      // - activate bumpers, cigar mode of the nimm2 robot
      // - create pointer to nimm2 (with odeHandle, osg Handle and configuration)
      // - place robot
      Nimm2Conf c = Nimm2::getDefaultConf();
      c.force = 4;
      c.bumper = true;
      c.cigarMode = true;
      vehicle = new Nimm2(odeHandle, osgHandle, c, ("Nimm2" + individual->getName()).c_str());
      vehicle->place(Pos((double) (ind % (int) sqrt(numberIndividuals)) * 19.0,
          19.0 * (double) (ind / (int) sqrt(numberIndividuals)), 0.0));

      // Read the gene values and create the neuron matrix.
      // The genes have a value of type IValue. We use only double values so we took for this interface
      // a TemplateValue<double> which is type of an IValue (see create prototypes in start()).
      // So we only need to cast them! Than we can read it!
      matrix::Matrix init(2, 2);
      double v1, v2, v3, v4;
      TemplateValue<double>* value = dynamic_cast<TemplateValue<double>*> (individual->getGen(0)->getValue());
      value != 0 ? v1 = value->getValue() : v1 = 0.0;
      value = dynamic_cast<TemplateValue<double>*> (individual->getGen(1)->getValue());
      value != 0 ? v2 = value->getValue() : v2 = 0.0;
      value = dynamic_cast<TemplateValue<double>*> (individual->getGen(2)->getValue());
      value != 0 ? v3 = value->getValue() : v3 = 0.0;
      value = dynamic_cast<TemplateValue<double>*> (individual->getGen(3)->getValue());
      value != 0 ? v4 = value->getValue() : v4 = 0.0;
      // set the matrix values
      init.val(0, 0) = v1;
      init.val(0, 1) = v2;
      init.val(1, 0) = v3;
      init.val(1, 1) = v4;

      // Create pointer to controller:
      // Push controller in global list of configurables.
      // Use the neuron matrix for the controller.
      InvertMotorNStepConf confMotorNStep = InvertMotorNStep::getDefaultConf();
      confMotorNStep.initialC = init;
      InvertMotorNStep *controller = new InvertMotorNStep(confMotorNStep);
      global.configs.push_back(controller);

      // create pointer to one2onewiring
      One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));

      // create pointer to agent
      // initialise pointer with controller, robot and wiring
      // push agent in global list of agents
      agent = new OdeAgent(plotoptions);
      agent->init(controller, vehicle, wiring);
      global.agents.push_back(agent);

      // create measure for the agent
      // and connect the measure with the fitness strategy
      std::list<Trackable*> trackableList;
      trackableList.push_back(vehicle);
      TrackableMeasure* trackableEntropy = new TrackableMeasure(trackableList, ("E Nimm2 of " + individual->getName()).c_str(),
              ENTSLOW, playground->getCornerPointsXY(), X | Y, 18);
      fitnessStr->m_storage.push_back(&trackableEntropy->getValueAddress());
      entropyMeasure.push_back(trackableEntropy);
      storageMeasure.push_back(trackableEntropy);
    }
  }

  /**
   * our fitness strategy
   */
  TemplateCycledGaSimulationFitnessStrategy* fitnessStr; // the fitness strategy

  /**
   * the number of robots inside
   */
  double numberIndividuals; // number of individuals

  /**
   * the actual needed and active entropy measures
   */
  std::vector<TrackableMeasure*> entropyMeasure; // all active measures for the entropy.

  /**
   * all entropy measures
   */
  std::vector<TrackableMeasure*> storageMeasure; // all measures for the entropy.

  /**
   * a mutation factor strategy for the mutation strategy (standard)
   */
  IMutationFactorStrategy* mutFaStr;

  /**
   * a mutation strategy (will be standard)
   */
  IMutationStrategy* mutStr;

  /**
   * a random strategy for the genes
   */
  IRandomStrategy* randomStr;
};

/**
 * our program!
 * @param argc command line argument counter
 * @param argv command line arguments
 * @return 0 if all OK or 1 if not
 */
int main(int argc, char **argv)
{
  int ret;
  ThisSim sim(9);
  ret = sim.run(argc, argv) ? 0 : 1;
  return ret;
}

