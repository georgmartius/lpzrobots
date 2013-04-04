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
 *   Revision 1.21  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.20  2009/08/11 12:30:39  robot12
 *   update the simstep variable from "this" to globalData! (guettler)
 *
 *   Revision 1.19  2009/08/03 08:03:09  guettler
 *   order of setTexture and setPosition for Primitives corrected
 *
 *   Revision 1.18  2009/08/03 08:01:02  guettler
 *   avoid using setColor when creating Playground
 *
 *   Revision 1.17  2009/08/03 07:57:04  guettler
 *   tiny compatibility mod (contains is moved)
 *
 *   Revision 1.16  2008/05/01 22:03:54  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.15  2007/12/06 10:02:49  der
 *   abstractground: returns now cornerpoints
 *   abstractobstacle: is now trackable
 *   hudstatistics: supports now AbstractmMeasure
 *
 *   Revision 1.14  2007/10/01 13:27:14  robot3
 *   made some improvements
 *
 *   Revision 1.13  2007/09/28 12:29:12  robot3
 *   loop test now working
 *
 *   Revision 1.12  2007/09/28 10:08:49  robot3
 *   fixed memory bugs, statistics are from now on aligned right
 *
 *   Revision 1.11  2007/09/28 09:15:25  robot3
 *   extended comments
 *
 *   Revision 1.10  2007/09/28 08:46:26  robot3
 *   added loop tests
 *
 *   Revision 1.9  2007/09/27 16:00:52  der
 *   made some tests
 *
 *   Revision 1.8  2007/09/27 10:44:33  robot3
 *   tested new WSM (WindowStatisticsManager)
 *
 *   Revision 1.7  2007/09/25 07:48:52  robot3
 *   made some tests with white noise
 *
 *   Revision 1.6  2007/05/09 14:57:25  robot3
 *   to increase or reduce the simulation speed (realtimefactor), use + and -
 *   to toggle to the maximum simulation speed, use * in the simulation
 *
 *   Revision 1.5  2007/05/08 10:18:15  der
 *   added a function for starting the measure after a given time.
 *   made some tests
 *
 *   Revision 1.4  2007/05/07 20:56:58  robot3
 *   testing new nice statistic tools
 *   made some tests about force "sensors"
 *
 *   Revision 1.3  2007/04/19 13:45:05  robot3
 *   modified for mi tests
 *
 *   Revision 1.2  2007/04/12 13:30:03  robot3
 *   tests
 *
 *   Revision 1.1  2007/03/22 08:03:35  robot3
 *   simulation for testing the mutual information of an arbitrary controller
 *
 *
 ***************************************************************************/
#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>

#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/passivecapsule.h>

#include <selforg/invertnchannelcontroller.h>
#include <selforg/invertmotornstep.h>
#include <selforg/invertmotorspace.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

#include <ode_robots/axisorientationsensor.h>

#include <ode_robots/hurlingsnake.h>
#include <ode_robots/schlangeservo2.h>
#include <ode_robots/caterpillar.h>
#include <ode_robots/nimm2.h>
#include <ode_robots/nimm4.h>
#include <ode_robots/sphererobot3masses.h>
#include <ode_robots/sliderwheelie.h>
#include <ode_robots/shortcircuit.h>

#include <selforg/discretecontrolleradapter.h>
#include <selforg/oneactivemultipassivecontroller.h>
#include <selforg/mutualinformationcontroller.h>

#include <selforg/statistictools.h>
#include <selforg/statisticmeasure.h>

#include <ode_robots/substance.h>

#include <stdio.h>
#include <string.h>


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

class ThisSim : public Simulation /*, public Inspectable*/
{
public:

  StatisticTools* stats;
  Nimm2* myNimm2;
  MutualInformationController* mic;

  double cInit;
  StatisticMeasure* convTest0;
  StatisticMeasure* convTest1;
  /*
  virtual std::list<iparamkey> getInternalParamNames() const  {
          std::list<iparamkey> list;
          list+=std::string("sumForce");
          return list;
  }

  virtual std::list<iparamval> getInternalParams() const {
          std::list<iparamval> list;
          //list+=getAvgOf3( oldest,old,myNimm2->getSumForce());
          return list;
  }
  */


  ThisSim(double cInit=1.0) : cInit(cInit) {}

  ~ThisSim() {}

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {

    int numNimm2=1;

    stats = new StatisticTools();

    setCameraHomePos(Pos(-19.15, 13.9, 6.9),  Pos(-126.1, -17.6, 0));
    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)

    global.odeConfig.setParam("noise",0.1);
    global.odeConfig.setParam("controlinterval",1);
    global.odeConfig.setParam("realtimefactor",0);
    //  global.odeConfig.setParam("simstepsize",0.1);
    //  global.odeConfig.setParam("drawinterval",5);
    // initialization

    Playground* playground =
      new Playground(odeHandle, osgHandle.changeColor(Color(0.88f,0.4f,0.26f,0.2f)),osg::Vec3(18, 0.2, 2.0));
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    Substance substance;
    substance.toRubber(40);
    playground->setGroundSubstance(substance);
    global.obstacles.push_back(playground);

    for(int i=0; i<0; i++)
    {
      PassiveSphere* s =
        new PassiveSphere(odeHandle,
                          osgHandle.changeColor(Color(184 / 255.0, 233 / 255.0, 237 / 255.0)), 0.2);
      s->setTexture("Images/dusty.rgb");
      s->setPosition(Pos(i*0.5-2, i*0.5, 1.0));
      global.obstacles.push_back(s);
    }

    for (int j=0;j<4;j++)
    {
      for(int i=0; i<4; i++)
      {
        PassiveBox* b =
          new PassiveBox(odeHandle,
                         osgHandle.changeColor(Color(1.0f,0.2f,0.2f,0.5f)), osg::Vec3(1.5+i*0.01,1.5+i*0.01,1.5+i*0.01),40.0);
        b->setTexture("Images/light_chess.rgb");
        b->setPosition(Pos(i*4-5, -5+j*4, 1.0));
        global.obstacles.push_back(b);
      }
    }

    for(int i=0; i<0; i++)
    {
      PassiveCapsule* c =
        new PassiveCapsule(odeHandle, osgHandle, 0.2f, 0.3f, 0.3f);
      c->setPosition(Pos(i-1, -i, 1.0));
      c->setColor(Color(0.2f,0.2f,1.0f,0.5f));
      c->setTexture("Images/light_chess.rgb");
      global.obstacles.push_back(c);
    }

    OdeAgent* agent;
    AbstractWiring* wiring;
    AbstractController *controller;

    Nimm2Conf nimm2conf = Nimm2::getDefaultConf();
    nimm2conf.size = 1.6;
    nimm2conf.force = 5;
    nimm2conf.speed=12;
    nimm2conf.cigarMode=true;
    nimm2conf.singleMotor=false;
    nimm2conf.visForce=true;
    nimm2conf.boxMode=true;
    for(int r=0; r < numNimm2; r++)
    {
      myNimm2 = new Nimm2(odeHandle, osgHandle, nimm2conf, "Nimm2_" + std::itos(r));
      //robot = new ShortCircuit(odeHandle,osgHandle,1,1);
      ((OdeRobot*)myNimm2)->place(Pos ((r-1)*5,5,0));
      InvertMotorNStepConf invertnconf = InvertMotorNStep::getDefaultConf();
      invertnconf.cInit = cInit;
      controller = new InvertMotorNStep(invertnconf);
      controller->setParam( "epsA",0);
      controller->setParam( "epsC",0);
      //         controller = new InvertMotorSpace(15);
      //      controller->setParam("s4avg",10);
      //controller = new SineController();
      //  controller->setParam( "sinerate",1);
      //    controller->setParam("factorB",0); // not needed here and it does some harm on the behaviour
      //wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      wiring = new One2OneWiring(new WhiteUniformNoise());
      agent = new OdeAgent( std::list<PlotOption>() );
      // create DiscreteControllerAdapter
      //            DiscreteControllerAdapter* discretesizer = new DiscreteControllerAdapter(controller);
      //            discretesizer->setIntervalCount(3);
      OneActiveMultiPassiveController* onamupaco = new OneActiveMultiPassiveController(controller,"main");
      mic = new MutualInformationController(30);
      mic->setParam("showF",0);
      mic->setParam("showP",0);
      onamupaco->addPassiveController(mic,"mi30");

      agent->addInspectable((Inspectable*)stats);
      agent->addCallbackable((Callbackable*)stats);
      agent->init(onamupaco, myNimm2                , wiring);
      global.configs.push_back(controller);
      global.agents.push_back(agent);

      stats->beginMeasureAt(100);

      stats->addMeasure(mic->getMI(0),"MI0",ID,0);
      stats->addMeasure(mic->getMI(1),"MI1",ID,0);

      // this->getWSM()->beginMeasureAt(100);
      this->getHUDSM()->addMeasure(mic->getMI(1),"MI 1",ID,1);
      this->getHUDSM()->addMeasure(mic->getMI(0),"MI 0",ID,1);

         convTest1=getHUDSM()->getMeasure( mic->getMI(1),"MI 1 CONV",CONV,50000,0.002);
      // getWSM()->addMeasure( mic->getMI(1),"MI 1 CONV",CONV,5,10.0);
         convTest0=getHUDSM()->getMeasure( mic->getMI(0),"MI 0 CONV",CONV,50000,0.002);



      stats->addMeasure(myNimm2->getSumForce(), "sumForce", ID, 3);
                  stats->addMeasure(myNimm2->getSumForce(), "sumForceAvg50", AVG, 50);
                  stats->addMeasure(myNimm2->getContactPoints(),"contactPoints",ID,0);
                  double& peakForce = stats->addMeasure(myNimm2->getSumForce(),"peakForce",PEAK,0,0.06333);
                  stats->addMeasure(peakForce, "peakForceMax", MAX, 0);
                  stats->addMeasure(myNimm2->getSumForce(), "ForceMax", MAX, 0);
                  double& sumsumForce = stats->addMeasure(peakForce, "sumPeakForce50", SUM, 50);
                  stats->addMeasure(sumsumForce, "sumPeakForceAvg50", AVG, 50);
                  stats->addMeasure(sumsumForce, "MaxsumPeakForce50", MAX, 50);


    }


    std::cout << "running with cInit = " << this->cInit << "." << std::endl;

  }

  /** optional additional callback function which is called every simulation step.
  Called between physical simulation step and drawing.
  @param draw indicates that objects are drawn in this timestep
  @param pause indicates that simulation is paused
  @param control indicates that robots have been controlled this timestep
   */
  void addCallback(GlobalData& globalData, bool draw, bool pause, bool control)
  {
    if (globalData.sim_step%100000==0) {
      printf("timeSteps   = %li\n",globalData.sim_step);
      printf("time in min = %f\n",((float)globalData.sim_step)/100/60);
      printf("MI sensor 0 = %f\n",mic->getMI(0));
      printf("MI sensor 1 = %f\n",mic->getMI(1));
    }
    if ((this->convTest0->getValue()==1.0)&&(this->convTest1->getValue()==1.0)) {
      FILE* file;
      char filename[256];
      sprintf(filename, "MI_C_%f.log", this->cInit);

      file = fopen(filename,"w");

      fprintf(file, "#Logfile for measuring the Mutual Information\n");
      fprintf(file,"timeSteps   = %li\n",globalData.sim_step);
      fprintf(file,"time in min = %f\n",((float)globalData.sim_step)/100/60);
      fprintf(file,"MI sensor 0 = %f\n",mic->getMI(0));
      fprintf(file,"MI sensor 1 = %f\n",mic->getMI(1));
      fflush(file);
      if(file) fclose(file);
      printf("timeSteps   = %li\n",globalData.sim_step);
      printf("time in min = %f\n",((float)globalData.sim_step)/100/60);
      printf("MI sensor 0 = %f\n",mic->getMI(0));
      printf("MI sensor 1 = %f\n",mic->getMI(1));
      simulation_time_reached=true;
   }
  }

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (down)
    { // only when key is pressed, not when released
      switch ( (char) key )
      {
      default:
        return false;
        break;
      }
    }
    return false;
  }

};

int main (int argc, char **argv)
{


  // check for cinit value
  int index = ThisSim::contains(argv, argc, "-cinit");
  if(index) {
    if(argc > index) {
      ThisSim sim(atof(argv[index]));
      sim.run(argc,argv);
    }
  } else for (double cinit=0.0;cinit<=2.1;cinit+=0.05)  {
    ThisSim sim;
    sim.cInit=cinit;
    sim.run(argc,argv);
  }
}
