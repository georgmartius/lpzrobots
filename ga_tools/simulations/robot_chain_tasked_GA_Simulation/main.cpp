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

// include all necessary stuff
#include <selforg/noisegenerator.h>
#include <ode_robots/odeagent.h>
#include <selforg/one2onewiring.h>
#include <ode_robots/nimm2.h>
#include <ode_robots/playground.h>
#include <selforg/invertmotorspace.h>
//#include <selforg/derivativewiring.h>
#include <selforg/invertmotornstep.h>

// include some needed files for parallel task handling
// class TaskedSimulation, holds the SimulationTaskHandle and additional info (like taskId)
#include <ode_robots/taskedsimulation.h>
// class SimulationTask encapsulates one simulation as a single task
#include <ode_robots/simulationtask.h>
// holds all data needed by handling the tasks, additionally there can be put more data.
#include <ode_robots/simulationtaskhandle.h>
// manages the handling of the tasks, including the parallel loop.
#include <ode_robots/simulationtasksupervisor.h>

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

#include "TemplateTaskedGaSimulationFitnessStrategy.h"

#include <selforg/trackablemeasure.h>
#include <selforg/statistictools.h>

// simple multithread api
#include <selforg/quickmp.h>

// mutual information
#include <selforg/oneactivemultipassivecontroller.h>
#include <selforg/mutualinformationcontroller.h>
#include <selforg/measureadapter.h>

#define NUMBER_GENERATION 15
#define NUMBER_OF_TESTS_BY_CALCULATE 120

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

// create your own SimulationTaskHandle
struct ThisSimulationTaskHandle : public SimulationTaskHandle {
    /*
     * only used one of this
     */
    union {
        /**
         * the individuals, which must be simulated
         */
        std::vector<Individual*>* individuals;

        /**
         * an array of double values, which should be simulated
         */
        double* array;
    };

    /**
     * the number of individuals, which should be simulated
     */
    int numberIndividuals;

    /**
     * should be an array simulated
     */
    bool isArraySet;

    /**
     * should be the best individual on the end of the algorithm sumulated
     */
    bool isBestAnimation;

    /**
     * should be one Individual be calculated
     */
    bool isCalculation;

    int numberElementsInSnake;

    /**
     * our fitness strategy
     */
    TemplateTaskedGaSimulationFitnessStrategy* fitnessStr; // the fitness strategy

    /**
     * if the isCalculation FLAG is set then here will the results standing
     *
     * must be prepared for NUMBER_OF_TESTS_BY_CALCULATE Elements!!!
     */
    double* entropies;

    /**
     * here will the results for the mutual information standing
     */
    double* mi;

    /**
     * here will the results for the h_x standing
     */
    double* hx;

    /**
     * here will the results for the h_yx standing
     */
    double* hyx;

    /**
     * here will the results for the h_yx standing
     */
    //double* hxsi;

    ThisSimulationTaskHandle() {
      isArraySet = false;
      isBestAnimation = false;
      isCalculation = false;
      numberElementsInSnake = 5;
    }
};

/**
 * Just create your own simulation, it's up to you.
 *
 * It's essential that your simulation is deduced from
 * TaskedSimulation instead of Simulation.
 * With this little change you have access to the
 * taskId and the global simTaskHandle.
 */
class ThisSim : public TaskedSimulation {
  public:

    OdeRobot* vehicle;
    OdeAgent* agent;

    /**
     * starting function (executed once at the beginning of the simulation loop/first cycle)
     * This function contains the additional parameters simTaskHandle and taskId, with these
     * you have access to your global data.
     */
    void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global,
        SimulationTaskHandle& sTHandle, int taskId) {

      ThisSimulationTaskHandle& handle = static_cast<ThisSimulationTaskHandle&> (sTHandle);

      if (handle.isBestAnimation || handle.isArraySet)
        global.odeConfig.setParam("realtimefactor", 1);
      else
        // set realtimefactor to maximum
        global.odeConfig.setParam("realtimefactor", 0);

      if (!handle.isArraySet && !handle.isCalculation)
        m_individual = (*handle.individuals)[taskId];

      // So we are now ready to start the algorithm!
      // But without the simulation we have no fun with the algorithm. ;) The only we just need is the simulation!
      // Also we must create the robots and agents for the simulation:
      createBots(global, handle, taskId);

      // First: position(x,y,z) second: view(alpha,beta,gamma)
      // gamma=0;
      // alpha == horizontal angle
      // beta == vertical angle
      //setCameraHomePos(Pos(-20.0, -20.0, 35.0), Pos(0., 0., 0.));
      setCameraHomePos(Pos(-34.0, 34.0, 15.0),  Pos(-135.0, -18.0, 0));
      // TODO: disable camera tracking (static (CameraManipulator) instead of CameraManipulatorTV)

      // initialisation
      // - set noise to 0.05
      global.odeConfig.noise = 0.05;

      if (!handle.isArraySet && !handle.isBestAnimation && !handle.isCalculation)
        std::cout << "Simulation " << taskId + 1 << " von " << handle.numberIndividuals << " aus Generation "
            << SingletonGenAlgAPI::getInstance()->getEngine()->getActualGenerationNumber() << " von "
            << NUMBER_GENERATION << " gestartet.\n";
    }

    /**
     * restart() is called at the second and all following starts of the cylce
     * The end of a cycle is determined by (simulation_time_reached==true)
     * @param the odeHandle
     * @param the osgHandle
     * @param globalData
     * @return if the simulation should be restarted; this is false by default
     */
    virtual bool restart(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global,
        SimulationTaskHandle& sTHandle, int taskId) {

      ThisSimulationTaskHandle& handle = static_cast<ThisSimulationTaskHandle&> (sTHandle);

      if (handle.isArraySet || handle.isBestAnimation)
        return false;

      //read the result
      double fitness = m_trackableEntropy->getValue();

      if(handle.isCalculation) {
        handle.entropies[taskId] = fitness;

        return false;
      }

      //give the result back.
      unsigned int individualId = (*handle.individuals)[taskId]->getID();

      if (handle.fitnessStr->m_storage.size() <= individualId)
        handle.fitnessStr->m_storage.resize(handle.fitnessStr->m_storage.size() + handle.numberIndividuals);
      handle.fitnessStr->m_storage[individualId] = fitness;

      handle.mi[taskId] = mic->getMI(1);
      handle.hx[taskId] = mic->getH_x(1);
      handle.hyx[taskId] = mic->getH_yx(1);
      //handle.hxsi[taskId] = mic->getH_Xsi(1);
      return false; // don't restart, just quit
      // see template_cycledSimulation for more info about usage
    }

    /** optional additional callback function which is called every simulation step.
     Called between physical simulation step and drawing.
     @param draw indicates that objects are drawn in this timestep
     @param pause always false (only called of simulation is running)
     @param control indicates that robots have been controlled this timestep
     */
    virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
      // for demonstration: set simsteps for one cycle to 60.000/currentCycle (10min/currentCycle)
      // if simulation_time_reached is set to true, the simulation cycle is finished
      if (globalData.sim_step >= (500 / this->currentCycle)) {
        simulation_time_reached = true;
      }

      // make a step in the measure
      if (m_trackableEntropy)
        m_trackableEntropy->step();

      if(draw)
      {
        FOREACH(std::list<Joint*>, joints,j)
        {
          (*j)->update();
        }
      }
    }

    // add own key handling stuff here, just insert some case values
    virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down,
        SimulationTaskHandle& sTHandle, int taskI) {
      //ThisSimulationTaskHandle* simTaskHandle = static_cast<ThisSimulationTaskHandle*> (&sTHandle);
      if (down) { // only when key is pressed, not when released
        switch ((char) key) {
          default:
            return false;
            break;
        }
      }
      return false;
    }

  private:
    Individual* m_individual;
    TrackableMeasure* m_trackableEntropy;
    std::list<Joint*> joints;
    MutualInformationController* mic;
    OneActiveMultiPassiveController* onamupaco;

    /**
     * This function creates the robots and agents for one simulation.
     * @param global
     */
    void createBots(GlobalData& global, ThisSimulationTaskHandle& sTHandle, int taskId) {
      OdeRobot * vehicle; // the robot
      OdeRobot * FirstVehicle; // the Head of the robots
      OdeAgent * agent; // the agent
      Playground * playground; // the playground for the created robot

      // Next we need a playground for the robot:
      // Use Playground as boundary:
      // - create pointer to playground (odeHandle contains things like world and space the
      //   playground should be created in; odeHandle is generated in simulation.cpp)
      // - setting geometry for each wall of playground:
      //   setGeometry(double length, double width, double  height)
      // - setting initial position of the playground: setPosition(double x, double y, double z)
      // - push playground in the global list of obstacles(global list comes from simulation.cpp)
      playground = new Playground(odeHandle, osgHandle,osg::Vec3(100, 0.2, 2.0));
      playground->setColor(Color(1.0f,0.4f,0.26f,1.0f));
      playground->setGroundTexture("Images/wood.rgb");
      playground->setGroundColor(Color(0.2f,0.7f,0.2f,1.0f));
      playground->setPosition(osg::Vec3(20,20,1.00f));
      // register playground in obstacles list
      global.obstacles.push_back(playground);

      // Use Nimm2 vehicle as robot:
      // - get default configuration for nimm2
      // - activate bumpers, cigar mode of the nimm2 robot
      // - create pointer to nimm2 (with odeHandle, osg Handle and configuration)
      // - place robot
      Nimm2Conf c = Nimm2::getDefaultConf();
      c.size = 1.6;
      c.force = 0.1;
      c.speed=20;
      c.cigarMode=true;
      c.singleMotor=false;
      c.visForce=true;
      c.boxMode=true;
      c.bumper=true;
      std::vector<OdeRobot*> robots(sTHandle.numberElementsInSnake);

      // Read the gene values and create the neuron matrix.
      // The genes have a value of type IValue. We use only double values so we took for this interface
      // a TemplateValue<double> which is type of an IValue (see create prototypes in start()).
      // So we only need to cast them! Than we can read it!
      matrix::Matrix init(2, 2);
      double* values = new double[4*sTHandle.numberElementsInSnake];
      if (!sTHandle.isArraySet && !sTHandle.isCalculation) {
        for(int xi=0;xi<4*sTHandle.numberElementsInSnake;xi++) {
          TemplateValue<double>* value = dynamic_cast<TemplateValue<double>*> (m_individual->getGen(xi)->getValue());
          value != 0 ? values[xi] = value->getValue() : values[xi] = 0.0;
        }
      } else {
        for(int xi=0;xi<4*sTHandle.numberElementsInSnake;xi++) {
          values[xi] = sTHandle.array[xi];
        }
      }

      for (int j=0; j<sTHandle.numberElementsInSnake; j++) {
        // set the matrix values
        init.val(0, 0) = values[4*j];
        init.val(0, 1) = values[4*j + 1];
        init.val(1, 0) = values[4*j + 2];
        init.val(1, 1) = values[4*j + 3];

        if (sTHandle.isArraySet || sTHandle.isCalculation)
          vehicle = new Nimm2(odeHandle, osgHandle, c, "Nimm2");
        else
          vehicle = new Nimm2(odeHandle, osgHandle, c, ("Nimm2" + m_individual->getName()).c_str());
        vehicle->place(Pos(j*(2.5f),0.0f,1.0f));

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

        robots[j]=vehicle;

        if(j==0) {
          FirstVehicle = vehicle;

          onamupaco = new OneActiveMultiPassiveController(controller,"main");
          mic = new MutualInformationController(30);
          MeasureAdapter* ma = new MeasureAdapter(mic);
          onamupaco->addPassiveController(ma,"mi30");
          agent->init(onamupaco, vehicle, wiring);
        }
        else
          agent->init(controller, vehicle, wiring);


        global.agents.push_back(agent);
      }
      for (int j=0; j<sTHandle.numberElementsInSnake-1; j++){
        Joint* joint = new BallJoint(robots[j]->getMainPrimitive(),
                                     robots[j+1]->getMainPrimitive(),
                                     Pos((j+0.5)*(2.5f)-1.0f,0.0f,0.48f)
                                    );
        joint->init(odeHandle,osgHandle,true,1.0/6);
        joints.push_back(joint);
      }

      delete[] values;

      if (!sTHandle.isArraySet || !sTHandle.isBestAnimation) {
        // create measure for the agent
        // and connect the measure with the fitness strategy
        std::list<Trackable*> trackableList;
        trackableList.push_back(FirstVehicle);
        m_trackableEntropy = new TrackableMeasure(trackableList, "E Nimm2", ENTSLOW, playground->getCornerPointsXY(), X | Y, 18);
      } else
        m_trackableEntropy = 0;
    }

};

/**
 * Defines a method to construct a ThisSim. This method is needed by the
 * SimulationTask, provided through the SimulationTaskSupervisor.
 * If you like to get the singleton instance of SimulationTaskSupervisor, you have
 * to pass as argument an instance of the ThisSimulationBuilder.
 */
class ThisSimCreator : public TaskedSimulationCreator {
  public:
    virtual TaskedSimulation* buildTaskedSimulationInstance() {
      return new ThisSim();
    }
};

int main(int argc, char **argv) {
  int numberIndividuals = 360;
  int mutationProbability = 50;  //5% in 1/1000 => 50
  double base = -100.0;
  double factor = 200.0;
  double epsilon = 0.0;
  int baseSize = (numberIndividuals - (numberIndividuals/10))/2;
  int numChildren = numberIndividuals - (baseSize*2);
  int numberElements = 5;
  int countGensIndex = Simulation::contains(argv, argc, "-gene_count");
  int newArgc = 0;
  char* newArgv[] = {};
  std::vector<double> mi;
  std::vector<double> hx;
  std::vector<double> hyx;
  //std::vector<double> hxsi;
  int geneRestore = Simulation::contains(argv, argc, "-gene_restore");
  FILE* restoreFile;

  // by reason of thread synchronizations effects we generate 2 threads per processor
  SimulationTaskSupervisor::getInstance()->setNumberThreadsPerCore(2);

  if (countGensIndex) {
    int countGens = atoi(argv[countGensIndex]);
    double* array = new double[countGens];

    for (int index = 0; index < countGens; index++) {
      //array[index]=atof(argv[countGensIndex+index]);
      double x = strtod(argv[countGensIndex + index + 1], NULL);
      array[index] = x;
    }

    // 1. create your own deduced SimulationTaskHandle
    ThisSimulationTaskHandle simTaskHandle;
    // 2. create your ThisSimCreator
    ThisSimCreator simCreator;
    // 3. set simTaskHandle and simCreator
    SimulationTaskSupervisor::setSimTaskHandle(simTaskHandle);
    SimulationTaskSupervisor::setTaskedSimCreator(simCreator);
    // 4. add needed data to your simTaskHandle
    simTaskHandle.numberIndividuals = 1;
    simTaskHandle.array = array;
    simTaskHandle.numberElementsInSnake = countGens / 4;

    if(Simulation::contains(argv, argc, "-gene_simulate")) {
      char* newArgv2[] = {"-nographics"};
      simTaskHandle.isCalculation = true;
      simTaskHandle.entropies = new double[NUMBER_OF_TESTS_BY_CALCULATE];
      newArgc = 1;
      // 5. create the SimulationTasks
      // just add another task pool and run this ones
      SimulationTaskSupervisor::getInstance()->createSimTasks(NUMBER_OF_TESTS_BY_CALCULATE);
      SimulationTaskSupervisor::getInstance()->setSimTaskNameSuffix("GeneSet");
      SimulationTaskSupervisor::getInstance()->runSimTasks(&newArgc, newArgv2);

      double fit;

      for(int i=0;i<NUMBER_OF_TESTS_BY_CALCULATE;i++) {
        fit += simTaskHandle.entropies[i];
        printf("Entropy: %3i ist %lf\n",i+1,simTaskHandle.entropies[i]);
      }

      fit /= NUMBER_OF_TESTS_BY_CALCULATE;

      printf("\n\nMITTEL:\t%lf\n",fit);

      delete[] simTaskHandle.entropies;
    }
    else {
      simTaskHandle.isArraySet = true;
      // 5. create the SimulationTasks
      // just add another task pool and run this ones
      SimulationTaskSupervisor::getInstance()->createSimTasks(1);
      SimulationTaskSupervisor::getInstance()->setSimTaskNameSuffix("GeneSet");
      SimulationTaskSupervisor::getInstance()->runSimTasks(&newArgc, newArgv);
    }

    delete[] array;

    return 0;
  }

  // ga_tool initialising
  // First we need some variables.

  RandGen random; // a random generator
  IFitnessStrategy* invertedFitnessStr; // the inverted fitness strategy
  IGenerationSizeStrategy* gSStr; // a generation size strategy
  ISelectStrategy* selStr; // a select strategy
  GenPrototype** pro = new GenPrototype*[4*numberElements]; // the n prototypes for the genes 2 Sensors - 2 Engines ==> 4 neuron connections
  TemplateTaskedGaSimulationFitnessStrategy* fitnessStr; // the fitness strategy
  IMutationFactorStrategy* mutFaStr;
  IMutationStrategy* mutStr;
  IRandomStrategy* randomStr;

  //this 3 PlotOptions are needed for some measures. They will bring us some data on the screen and save all to a log file.
  PlotOption opt1(GuiLogger); // a plot Option for the generation measure to guilogger
  PlotOption opt2(File); // a plot Option for the generation measure to file
  PlotOption optGen(File); // a plot Option for gene measure to file
  //PlotOption optMi(File); // a plot Option for the Mutual Information to a file
  //PlotOptionEngine* optEngineMI(optMI); // a plot Option Engine for the Mutual Information

  //optEngineMI->addPlotOption(optMI);

  // Next we need the general strategies for the algorithm.
  // - a GenerationSizeStrategy: Here we take a fixed size strategy. This means every generation has the size of "numberIndividuals"
  // - a SelectStrategy: Here we take a tournament strategy which tests 2 individuals. The better one will win.
  gSStr = SingletonGenAlgAPI::getInstance()->createFixGenerationSizeStrategy(baseSize);
  SingletonGenAlgAPI::getInstance()->setGenerationSizeStrategy(gSStr);
  selStr = SingletonGenAlgAPI::getInstance()->createTournamentSelectStrategy(&random);
  SingletonGenAlgAPI::getInstance()->setSelectStrategy(selStr);

  // After this we need the fitness strategy.
  // Here we need our own strategy! But our strategy will be higher if the individual are better.
  // So we need a inverted fitness strategy because the genetic algorithm will optimise again zero.
  // More details on this strategies can be found in the belonging header files.  // More details on this strategies can be found in the belonging header files.
  fitnessStr = new TemplateTaskedGaSimulationFitnessStrategy();
  invertedFitnessStr = SingletonGenAlgAPI::getInstance()->createInvertedFitnessStrategy(fitnessStr);
  SingletonGenAlgAPI::getInstance()->setFitnessStrategy(invertedFitnessStr);

  // Now its time to create all needed stuff for the genes.
  // - mutation strategy for the prototypes
  // - random strategy for the prototypes
  // - and the 4 prototypes for the genes:
  mutFaStr = SingletonGenAlgAPI::getInstance()->createStandartMutationFactorStrategy();
  // The second value means the mutation probability in 1/1000. Normal is a value lower than max. 5%.
  mutStr = SingletonGenAlgAPI::getInstance()->createValueMutationStrategy(mutFaStr, mutationProbability);
  // The last parameters ensure that the created genes lay inside the interval from -100 to +100.
  randomStr = SingletonGenAlgAPI::getInstance()->createDoubleRandomStrategy(&random, base, factor, epsilon);
  // The prototypes need a name, a random strategy to create random genes and a mutation strategy to mutate existing genes.
  for(int xi = 0; xi<4*numberElements; xi++) {
    char buffer[10];
    sprintf(buffer,"P%i",xi+1);
    pro[xi] = SingletonGenAlgAPI::getInstance()->createPrototype(buffer, randomStr, mutStr);
    SingletonGenAlgAPI::getInstance()->insertGenPrototype(pro[xi]);
  }

  // At last we create all interesting measures (PlotOptions).
  opt1.setName("opt1");
  opt2.setName("opt2");
  SingletonGenAlgAPI::getInstance()->enableMeasure(opt1);
  SingletonGenAlgAPI::getInstance()->enableMeasure(opt2);
  optGen.setName("optGen");
  SingletonGenAlgAPI::getInstance()->enableGenContextMeasure(optGen);

  // Prepare the first generation:
  //   We can use "run" for a automatically run or we must control all ourself like here!
  //   So we must prepare the first generation, for this the algorithm must know how many individuals he should create,
  //   how much will die on the end and if he should make an automatically update of the statistic values.
  //   The automatically update isn't possible because before we need a run of the simulation, so we make it later ourself (param false)!
  //   restore if needed
  // or restore from file
  int start=0;
  if(geneRestore) {
    restoreFile = fopen(argv[geneRestore],"rb");
    //dont check file open!
    if(!SingletonGenAlgAPI::getInstance()->restoreGA(restoreFile)) {
      return 1;
    }
    start = SingletonGenAlgAPI::getInstance()->getEngine()->getActualGenerationNumber()-1;
    geneRestore=0;
    fclose(restoreFile);

    //restore fitness value range inside the fitness strategy
    fitnessStr->m_storage.resize(SingletonGenAlgAPI::getInstance()->getEngine()->getNumIndividual());
  } else {
    SingletonGenAlgAPI::getInstance()->prepare(baseSize, numChildren, &random, false);
  }

  // 1. create your own deduced SimulationTaskHandle
  ThisSimulationTaskHandle simTaskHandle;
  // 2. create your ThisSimCreator
  ThisSimCreator simCreator;
  // 3. set simTaskHandle and simCreator
  SimulationTaskSupervisor::setSimTaskHandle(simTaskHandle);
  SimulationTaskSupervisor::setTaskedSimCreator(simCreator);
  // 4. add needed data to your simTaskHandle
  simTaskHandle.fitnessStr = fitnessStr;

  // generation iterating
  for (int x = start; x < NUMBER_GENERATION; x++) {

    // 4. add needed data to your simTaskHandle
    std::vector<Individual*>* individualVectorTemp =
        SingletonGenAlgAPI::getInstance()->getEngine()->getActualGeneration()->getAllUnCalculatedIndividuals();
    simTaskHandle.individuals = individualVectorTemp;
    simTaskHandle.numberIndividuals = individualVectorTemp->size();
    simTaskHandle.numberElementsInSnake = numberElements;
    simTaskHandle.mi = new double[individualVectorTemp->size()];
    simTaskHandle.hx = new double[individualVectorTemp->size()];
    simTaskHandle.hyx = new double[individualVectorTemp->size()];
    //simTaskHandle.hxsi = new double[individualVectorTemp->size()];

    // 5. create the SimulationTasks
    // just add another task pool and run this ones
    char buffer[15];
    sprintf(buffer, "taskpool %i", x);
    SimulationTaskSupervisor::getInstance()->createSimTasks(individualVectorTemp->size());
    SimulationTaskSupervisor::getInstance()->setSimTaskNameSuffix(buffer);

    printf("Starte %i Threads.\n", (int) individualVectorTemp->size());

    SimulationTaskSupervisor::getInstance()->runSimTasks(&argc, argv);

    //    QMP_BARRIER();

    for(int ij=0;ij<individualVectorTemp->size();ij++) {
      mi.push_back(simTaskHandle.mi[ij]);
      hx.push_back(simTaskHandle.hx[ij]);
      hyx.push_back(simTaskHandle.hyx[ij]);
      //hxsi.push_back(simTaskHandle.hxsi[ij]);
    }

    delete[] simTaskHandle.mi;
    delete[] simTaskHandle.hx;
    delete[] simTaskHandle.hyx;
    //delete[] simTaskHandle.hxsi;

    for(int d=0;d<individualVectorTemp->size();d++) {
      (*individualVectorTemp)[d]->getFitness();
    }

    delete individualVectorTemp;

    //for safety store the GA
    restoreFile = fopen("store.dat","wb");
    if(restoreFile!=NULL){
      SingletonGenAlgAPI::getInstance()->storeGA(restoreFile);
      fclose(restoreFile);
    }
    else
      printf("[ERROR] by open the store file.\n");

    RandGen random; // a random generator

    // Step in the algorithm:
    // - update the statistical values inside the genetic algorithm
    // - make a step in the measure
    // - select the individual which will be killed by use of their statistical values.
    // - and generate new individuals
    SingletonGenAlgAPI::getInstance()->update();
    SingletonGenAlgAPI::getInstance()->measureStep(x + 1);

    if (x < NUMBER_GENERATION - 1) {
      SingletonGenAlgAPI::getInstance()->select();
      SingletonGenAlgAPI::getInstance()->crossover(&random);
    }

    FILE* file2 = fopen("indTest.txt", "a");
    if (file2 != NULL) {
      fprintf(file2, "############################\n");
      fprintf(file2, "%s", SingletonGenAlgAPI::getInstance()->getEngine()->getAllIndividualAsString().c_str());
      fclose(file2);
    }

    file2 = fopen("verTest.txt", "a");
    if (file2 != NULL) {
      fprintf(file2, "############################\n");
      fprintf(file2, "%s", SingletonGenAlgAPI::getInstance()->getEngine()->getIndividualRoot().c_str());
      fclose(file2);
    }

    // all information about the mutual information
    file2 = fopen("miTest.txt", "a");
    if(file2 != NULL) {
      fprintf(file2, "############################\n");
      for(int hy=0;hy<mi.size();hy++) {
        fprintf(file2,"%-.12lf\t%-.12lf\t%-.12lf\n",mi[hy],hx[hy],hyx[hy]);
      }
      fclose(file2);
    }
  }

  FILE* file = fopen("ind.txt", "w");
  if (file != NULL) {
    fprintf(file, "%s", SingletonGenAlgAPI::getInstance()->getEngine()->getAllIndividualAsString().c_str());
    fclose(file);
  }

  file = fopen("ver.txt", "w");
  if (file != NULL) {
    fprintf(file, "%s", SingletonGenAlgAPI::getInstance()->getEngine()->getIndividualRoot().c_str());
    fclose(file);
  }

  // all information about the mutual information
  file = fopen("mi.txt", "w");
  if(file != NULL) {
    for(int hy=0;hy<mi.size();hy++) {
      fprintf(file,"%-.12lf\t%-.12lf\t%-.12lf\n",mi[hy],hx[hy],hyx[hy]);
    }
    fclose(file);
  }

  if (Simulation::contains(argv, argc, "-genes_best")) {
    // 1. create your own deduced SimulationTaskHandle
    ThisSimulationTaskHandle simTaskHandle;
    // 2. create your ThisSimCreator
    ThisSimCreator simCreator;
    // 3. set simTaskHandle and simCreator
    SimulationTaskSupervisor::setSimTaskHandle(simTaskHandle);
    SimulationTaskSupervisor::setTaskedSimCreator(simCreator);
    // 4. add needed data to your simTaskHandle
    simTaskHandle.numberIndividuals = 1;
    simTaskHandle.individuals = new std::vector<Individual*>();
    simTaskHandle.individuals->push_back(SingletonGenAlgAPI::getInstance()->getBestIndividual());
    simTaskHandle.isBestAnimation = true;
    simTaskHandle.numberElementsInSnake = numberElements;
    // 5. create the SimulationTasks
    // just add another task pool and run this ones
    SimulationTaskSupervisor::getInstance()->createSimTasks(1);
    SimulationTaskSupervisor::getInstance()->setSimTaskNameSuffix("GeneSet");
    SimulationTaskSupervisor::getInstance()->runSimTasks(&newArgc, newArgv);

    delete simTaskHandle.individuals;
  }

  printf("\n\nRESULT:\t%s\n\n", SingletonGenAlgAPI::getInstance()->getBestIndividual()->IndividualToString().c_str());

  //delete fitnessStr;
  SingletonGenAlgAPI::destroyAPI(true);
  delete[] pro;

  return 0;
}
