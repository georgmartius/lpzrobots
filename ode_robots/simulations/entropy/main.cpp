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
 *   Revision 1.6  2007-12-10 14:13:59  der
 *   actual version
 *
 *   Revision 1.5  2007/12/07 10:51:42  der
 *   tested some things
 *
 *   Revision 1.4  2007/12/07 08:51:18  der
 *   made some test
 *
 *   Revision 1.3  2007/12/06 19:38:17  der
 *   tested entropy
 *
 *   Revision 1.2  2007/12/06 15:58:31  der
 *   changed minor things
 *
 *   Revision 1.1  2007/12/06 10:01:18  der
 *   new simulation for evaluating of the entropy over the positin histogramm of the robot
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
#include "simulation.h"

#include "odeagent.h"
#include "playground.h"

#include "passivebox.h"

#include "nimm2.h"

#include <selforg/invertmotornstep.h>
#include <selforg/one2onewiring.h>

#include <selforg/oneactivemultipassivecontroller.h>
#include <selforg/mutualinformationcontroller.h>

#include <selforg/statistictools.h>
#include <selforg/statisticmeasure.h>
#include <selforg/trackablemeasure.h>

#include "substance.h"

#include <stdio.h>
#include <string.h>


using namespace lpzrobots;

class ThisSim : public Simulation {
public:

  StatisticTools* stats;
  Nimm2* myNimm2;
  MutualInformationController* mic;

  double cInit;
  StatisticMeasure* convTest0;
  StatisticMeasure* convTest1;
  TrackableMeasure* trackableEntropySLOW;

  ThisSim(double cInit=1.0) : cInit(cInit) {}

  ~ThisSim() {
    if (convTest0)
      delete(convTest0);
    if (convTest1)
      delete(convTest1);
    if (trackableEntropySLOW)
      delete(trackableEntropySLOW);
    if (stats)
      delete(stats);
    if (mic)
      delete(mic);
   }

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    int numNimm2=1;
    setCameraHomePos(Pos(-19.15, 13.9, 6.9),  Pos(-126.1, -17.6, 0));

    global.odeConfig.setParam("noise",0.05);
    global.odeConfig.setParam("realtimefactor",0);

    Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(18, 0.2, 2.0));
    playground->setColor(Color(0.88f,0.4f,0.26f,0.2f));
    playground->setPosition(osg::Vec3(0,0,0));
    Substance substance;
    substance.toRubber(40);
    playground->setGroundSubstance(substance);
    global.obstacles.push_back(playground);

    for (int j=0;j<4;j++)
      for(int i=0; i<4; i++) {
        PassiveBox* b =
          new PassiveBox(odeHandle,
                         osgHandle, osg::Vec3(1.5+i*0.01,1.5+i*0.01,1.5+i*0.01),0.0);
        b->setPosition(Pos(i*4-6, -5+j*4, 0.01));
        b->setColor(Color(1.0f,0.2f,0.2f,0.5f));
        b->setTexture("Images/light_chess.rgb");
        global.obstacles.push_back(b);
      }

    OdeAgent* agent;
    AbstractWiring* wiring;
    AbstractController *controller;

    Nimm2Conf nimm2conf = Nimm2::getDefaultConf();
    nimm2conf.size = 1.6;
    nimm2conf.force = 5;
    nimm2conf.speed=10;
    nimm2conf.cigarMode=true;
    nimm2conf.singleMotor=false;
    nimm2conf.visForce=true;
    nimm2conf.boxMode=true;
    
    for(int r=0; r < numNimm2; r++)
    {
      myNimm2 = new Nimm2(odeHandle, osgHandle, nimm2conf, "Nimm2_" + std::itos(r));
      ((OdeRobot*)myNimm2)->place(Pos ((r-1)*5,5,0));
      
      InvertMotorNStepConf invertnconf = InvertMotorNStep::getDefaultConf();
      invertnconf.cInit = cInit;
      controller = new InvertMotorNStep(invertnconf);
      controller->setParam( "epsA",0);
      controller->setParam( "epsC",0);
      
      wiring = new One2OneWiring(new WhiteNormalNoise());
      OneActiveMultiPassiveController* onamupaco = new OneActiveMultiPassiveController(controller,"main");
      mic = new MutualInformationController(30);
      mic->setParam("showF",0);
      mic->setParam("showP",0);
      onamupaco->addPassiveController(mic,"mi30");

      stats = new StatisticTools();
      agent = new OdeAgent( std::list<PlotOption>() );
      agent->addInspectable((Inspectable*)stats);
      agent->addCallbackable((Callbackable*)stats);
      agent->init(onamupaco, myNimm2		, wiring);
      //agent->init(controller, myNimm2		, wiring);
      char cdesc[6];
      sprintf(cdesc, "c=%f_",cInit);
      agent->setTrackOptions(TrackRobot(true,false, false, false,cdesc,10));
      global.configs.push_back(controller);
      global.agents.push_back(agent);

      
      convTest1=stats->getMeasure( mic->getMI(1),"MI 1 CONV",CONV,50000,0.01);
      convTest0=stats->getMeasure( mic->getMI(0),"MI 0 CONV",CONV,50000,0.01);
      
      //  stats->beginMeasureAt(100);

      //this->getHUDSM()->addMeasure(mic->getMI(1),"MI 1",ID,1);
      //this->getHUDSM()->addMeasure(mic->getMI(0),"MI 0",ID,1);

      std::list<Trackable*> trackableList;
      trackableList.push_back(myNimm2);
      trackableEntropySLOW= new TrackableMeasure(trackableList,"Eslow Nimm2",ENTSLOW,playground->getCornerPointsXY(),X | Y, 10);
      this->getHUDSM()->addMeasure(trackableEntropySLOW);
      //TrackableMeasure* trackableEntropy = new TrackableMeasure(trackableList,"E upd Nimm2",ENT,playground->getCornerPointsXY(),X | Y, 50);
      //this->getHUDSM()->addMeasure(trackableEntropy);
    
    }

    showParams(global.configs);
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
    if (this->sim_step%100000==0)
    {
      printf("timeSteps   = %li\n",this->sim_step);
      printf("time in min = %f\n",((float)this->sim_step)/100/60);
      printf("(MI0+MI1)/2 = %f\n",(mic->getMI(0)+mic->getMI(1))/2);
      //printf("Entropy     = %f\n",trackableEntropySLOW->getValue());
    }
    //if (this->sim_step>=600000)
    if  ((this->convTest0->getValue()==1.0)&&(this->convTest1->getValue()==1.0))
    {
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

void runSim(double cinit, int runs, int argc, char **argv) {
  double sum = 0.0;
  double mi=0.0;
  for (int i=0;i<runs;i++) {
    std::cout << "run number " << (i+1) << "..." << std::endl;
    ThisSim* sim;
    sim = new ThisSim(cinit);
    sim->run(argc,argv);
    sum+= sim->trackableEntropySLOW->getValue();
    mi = (sim->mic->getMI(0) + sim->mic->getMI(1))/2;
    delete(sim);
  }
  
  double avg = sum / ((double)runs);
  FILE* file;
  char filename[256];
  sprintf(filename, "ent_C.log");
  file = fopen(filename,"a");
  fprintf(file,"%f, %f, %f\n",cinit,avg,mi);
  fflush(file);  
}

int main (int argc, char **argv)
{
  // check for runs value
  int runs=1;
  int index = contains(argv, argc, "-runs");
  if(index &&  (argc > index))
      runs = atoi(argv[index]);
  // check for cinit value
  index = contains(argv, argc, "-cinit");
  if (index && (argc > index))
    {
      ThisSim sim(atof(argv[index]));
      sim.run(argc,argv);
    }
  else
  {
    for (double cinit=0.0;cinit<=0.9;cinit+=0.05)
      runSim(cinit,runs,argc,argv);
    for (double cinit=0.95;cinit<=1.15;cinit+=0.025)
      runSim(cinit,runs,argc,argv);
    for (double cinit=1.2;cinit<=1.3;cinit+=0.05)
      runSim(cinit,runs,argc,argv);
  }
}
