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
 *   Revision 1.24  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.23  2011/04/28 09:46:27  martius
 *   *** empty log message ***
 *
 *   Revision 1.22  2011/03/22 16:43:12  guettler
 *   - adpaptions to enhanced configurable and inspectable interface
 *   - agents are always in globalData.configs
 *   - showParams now done by simulations base code instead manually called
 *     in method start()
 *   - showParams(ConfigList&) is marked as deprecated
 *   - Configurable inheritance of Simulation moved to Base, Base is no longer derived
 *     from BackCaller (because Configurable is derived from BackCaller)
 *   - removed some old deprecated member lists in base
 *
 *   Revision 1.21  2011/01/31 11:31:10  martius
 *   renamed sox to soml
 *
 *   Revision 1.20  2011/01/31 10:46:11  martius
 *   made it compile again
 *
 *   Revision 1.19  2010/11/26 12:21:13  guettler
 *   - Configurable interface now allows to set bounds of paramval and paramint
 *     * setting bounds for paramval and paramint is highly recommended (for QConfigurable (Qt GUI).
 *   - minor bugfixes
 *
 *   Revision 1.18  2010/03/10 13:54:29  guettler
 *   some tiny changes
 *
 *   Revision 1.17  2010/03/10 08:54:27  guettler
 *   adjusted simelation params to get settins used in paper
 *
 *   Revision 1.16  2009/08/10 07:49:44  guettler
 *   tests with entropy
 *
 *   Revision 1.15  2009/05/04 11:10:29  guettler
 *   minor changes (prints)
 *
 *   Revision 1.14  2009/04/22 17:16:36  jhoffmann
 *   tested the cool cvs system how it works
 *
 *   Revision 1.13  2009/03/26 18:25:23  martius
 *   removed contains helper since it is in Base class
 *
 *   Revision 1.12  2009/03/25 15:44:23  guettler
 *   ParallelSplitShadowMap: corrected light direction (using directional light), complete ground is now shadowed
 *
 *   Revision 1.11  2008/05/01 22:03:54  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.10  2008/04/24 11:28:08  der
 *   final entropy sim
 *
 *   Revision 1.7  2008/01/17 09:54:31  der
 *   newest version contains a nimm2 train
 *
 *   Revision 1.6  2007/12/10 14:13:59  der
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
#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>

#include <ode_robots/passivebox.h>

#include <ode_robots/nimm2.h>

#include <selforg/invertmotornstep.h>
#include <selforg/invertmotorspace.h>
#include <selforg/one2onewiring.h>

#include <selforg/oneactivemultipassivecontroller.h>
#include <selforg/mutualinformationcontroller.h>

#include <selforg/statistictools.h>
#include <selforg/statisticmeasure.h>
#include <selforg/trackablemeasure.h>
#include <selforg/measureadapter.h>

#include <ode_robots/substance.h>

#include <stdio.h>
#include <string.h>


using namespace lpzrobots;
using namespace osg;


int contains(char **list, int len,  const char *str) {
  for(int i=0; i<len; i++) {
    if(strcmp(list[i],str) == 0)
      return i+1;
  }
  return 0;
}

class ThisSim : public Simulation
{
public:

  StatisticTools* stats;
  Nimm2* nimm2;
  MutualInformationController* mic;
  std::list<Joint*> joints;
  bool connectRobots;

  double cInit;
  double bInit;
  StatisticMeasure* convTest0;
  StatisticMeasure* convTest1;
  StatisticMeasure* convTest2;
  StatisticMeasure* convTest3;
  StatisticMeasure* convTest4;
  StatisticMeasure* convTest5;
  TrackableMeasure* trackableEntropy;
  TrackableMeasure* trackableEntropySLOW;

  ThisSim(double cInit=.1, double binit=0.0) : cInit(cInit), bInit(binit)
  {
    setCaption("LpzRobots Simulator          Martius, Güttler, Der");
  }

  ~ThisSim()
  {
    /* if (convTest0)
      delete(convTest0);
    if (convTest1)
      delete(convTest1);*/
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

    int number_x=5;
    int number_y=1;
    connectRobots = true;
    double distance = 1.1;

    setCameraHomePos(Pos(-76.7927, 49.4669, 42.7545),  Pos(-124.513, -28.5595, 0));
    setCameraMode(Follow);

    global.odeConfig.setParam("noise",0.05);
    global.odeConfig.setParam("realtimefactor",2);
    global.odeConfig.setParam("gravity",-9);

    Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(50, 0.2, 2.0));
    playground->setColor(Color(1.0f,0.4f,0.26f,1.0f));
    playground->setGroundTexture("Images/wood.rgb");
    playground->setGroundColor(Color(0.2f,0.7f,0.2f,1.0f));
    //playground->setGroundColor(Color(0.2f,0.7f,0.2f,1.0f));
    Substance substance;
    //substance.toSnow(0.05);
    substance.toRubber(20);
    playground->setPosition(osg::Vec3(0,0,1.00f));
    playground->setGroundSubstance(substance);
    global.obstacles.push_back(playground);
    double xboxes=0.0;
    double yboxes=0.0;
    /*    double xboxes=6.0;
    double yboxes=6.0;*/
    double boxdis=4.8;
    for (double j=0.0;j<xboxes;j++)
      for(double i=0.0; i<yboxes; i++)
      {
        double xsize=1.5;
        double ysize=1.5;
        double zsize=1.5;
        PassiveBox* b =
          new PassiveBox(odeHandle,
                         osgHandle, osg::Vec3(xsize,ysize,zsize),0.0);
        b->setPosition(Pos(boxdis*(i-(xboxes-1)/2.0),boxdis*(j-(yboxes-1)/2.0), 1.01));
        b->setColor(Color(1.0f,0.2f,0.2f,0.5f));
        b->setTexture("Images/light_chess.rgb");
        global.obstacles.push_back(b);
      }

    stats = new StatisticTools();

    OdeAgent* agent;
    AbstractWiring* wiring;
    AbstractController *controller;
    OdeRobot* nimm2;
    std::list<ComplexMeasure*> hmlist;
    std::list<ComplexMeasure*> mimlist;
    std::list<Trackable*> trackableList;

    std::vector<OdeRobot*> robots(number_x);
    for (int i=-0; i<number_y; i++)
    {
      for (int j=-0; j<number_x; j++)
      {
        //      nimm2 = new Nimm2(odeHandle);
        Nimm2Conf nimm2conf = Nimm2::getDefaultConf();
        nimm2conf.size = 1.6;
        nimm2conf.force = 6;
        nimm2conf.speed=20;
        nimm2conf.cigarMode=true;
        nimm2conf.singleMotor=false;
        nimm2conf.boxMode=true;
        nimm2conf.visForce =true;
        nimm2conf.bumper=true;
        wiring = new One2OneWiring(new WhiteNormalNoise());
        InvertMotorNStepConf invertnconf = InvertMotorNStep::getDefaultConf();
//        invertnconf.cInit = cInit;///////////////////////// cInit;
//        invertnconf.cNonDiagAbs=cNonDiag;
        invertnconf.initialC = matrix::Matrix(2,2);
        invertnconf.initialC.val(0,0)= cInit;
        invertnconf.initialC.val(0,1)= bInit;
        invertnconf.initialC.val(1,0)= bInit;
        invertnconf.initialC.val(1,1)= cInit;
        controller = new InvertMotorNStep(invertnconf);
        //if (j==2)
        //  nimm2conf.irFront = true;
        if ((i==0) && (j==0))
        {
          //nimm2conf.irBack = true;
          agent = new OdeAgent(global);
          nimm2 = new Nimm2(odeHandle, osgHandle, nimm2conf, "Nimm2Yellow");
          nimm2->setColor(Color(1.0,1.0,0));
          trackableList.push_back(nimm2);
          global.configs.push_back(controller);

          OneActiveMultiPassiveController* onamupaco = new OneActiveMultiPassiveController(controller);
//          mic = new MutualInformationController(1, -1, 1, true, true);
          mic = new MutualInformationController(30, -1, 1, true, true);

          MeasureAdapter* ma = new MeasureAdapter(mic);
          onamupaco->addPassiveController(ma);
          agent->addInspectable((Inspectable*)stats);
          agent->addCallbackable((Callbackable*)stats);
          agent->init(onamupaco, nimm2, wiring);
          //controller->setParam("epsC", 0.001);
          //controller->setParam("epsA", 0.0);
          char cdesc[32];
          sprintf(cdesc, "c=%f_",cInit);
          // agent->setTrackOptions(TrackRobot(true,false, false,false,cdesc,10));
          //  hmlist = ma->addSensorComplexMeasure("$H(x)",ENTSLOW,30,1);
          // mimlist = ma->addSensorComplexMeasure("$MI(x)",MI,30,1);
        }
        else
        {
          agent = new OdeAgent(global, PlotOption(NoPlot));
          nimm2 = new Nimm2(odeHandle, osgHandle, nimm2conf, "Nimm2_" + std::itos(i) + "_" + std::itos(j));
          agent->init(controller, nimm2, wiring);
          controller->setParam("epsC", 0.00);
          controller->setParam("epsA", 0.00);
          global.configs.push_back(controller);
        }
        if ((i==0) && (j==1))
          setWatchingAgent(agent);
        nimm2->place(Pos(j*(1.5+distance),i*1.26,1.0f));
        global.agents.push_back(agent);
        robots[j]=nimm2;
      }
      if(connectRobots)
        for(int j=0; j<number_x-1; j++)
        {
          Primitive* p1 = robots[j]->getMainPrimitive();
          Primitive* p2 = robots[j+1]->getMainPrimitive();
          Joint* joint = new BallJoint(p1,p2,(p1->getPosition()+p2->getPosition())/2.0);

          // Joint* joint = new BallJoint(robots[j]->getMainPrimitive(),
          //                              robots[j+1]->getMainPrimitive(),
          //                              Pos((j+0.5)*(1.5+distance)-1.0,i*1.26,1.48)
          //                             );
          joint->init(odeHandle,osgHandle,true,distance/6);
          joints.push_back(joint);
        }
    }
    //this->getHUDSM()->addMeasure(mic->getMI(0),"MI"/* 0*/,ID,1);
    //double& stepdiff = stats->addMeasure(mic->getMI(1),"MI NSTEPDIFF",NORMSTEPDIFF,1);
    //this->getHUDSM()->addMeasure(stepdiff,"MI DIFFAVG",MOVAVG,1000);

/*    this->getHUDSM()->addMeasure(mic->getH_x(1),"H(x) 1",ID,1);
    this->getHUDSM()->addMeasure(mic->getH_x(0),"H(x) 0",ID,1);
    this->getHUDSM()->addMeasure(mic->getH_yx(1),"H(y|x) 1",ID,1);
    this->getHUDSM()->addMeasure(mic->getH_yx(0),"H(y|x) 0",ID,1);*/
    //  this->getHUDSM()->addMeasureList(hmlist);
    //this->getHUDSM()->addMeasureList(mimlist);

    /*   convTest1=stats->getMeasure( mic->getMI(1),"MI 1 CONV",CONV,50000,0.001);
    convTest0=stats->getMeasure( mic->getMI(0),"MI 0 CONV",CONV,50000,0.001);
    convTest5=stats->getMeasure( mic->getH_x(1),"H(x) 1 CONV",CONV,50000,0.001);
    convTest4=stats->getMeasure( mic->getH_x(0),"H(x) 0 CONV",CONV,50000,0.001);
    convTest3=stats->getMeasure( mic->getH_yx(1),"H(y|x) 1 CONV",CONV,50000,0.001);
    convTest2=stats->getMeasure( mic->getH_yx(0),"H(y|x) 0 CONV",CONV,50000,0.001);*/

   // trackableEntropySLOW= new TrackableMeasure(trackableList,"E Nimm2 o(n2)",ENTSLOW,playground->getCornerPointsXY(),X | Y, 18);
//    trackableEntropy= new TrackableMeasure(trackableList,"E Nimm2 O(1)",ENT,playground->getCornerPointsXY(),X | Y, 20000);
    //this->getHUDSM()->addMeasure(trackableEntropySLOW);
   // this->getHUDSM()->addMeasure(trackableEntropySLOW);
    //TrackableMeasure* trackableEntropy = new TrackableMeasure(trackableList,"E upd Nimm2",ENT,playground->getCornerPointsXY(),X | Y, 50);
    //this->getHUDSM()->addMeasure(trackableEntropy);


    std::cout << "running with cDiag = " << this->cInit << "." << std::endl;
    std::cout << "running with bNonDiag = " << this->bInit << "." << std::endl;

  }

  /** optional additional callback function which is called every simulation step.
  Called between physical simulation step and drawing.
  @param draw indicates that objects are drawn in this timestep
  @param pause indicates that simulation is paused
  @param control indicates that robots have been controlled this timestep
   */
  void addCallback(GlobalData& globalData, bool draw, bool pause, bool control)
  {
    if(draw && connectRobots)
    {
      FOREACH(std::list<Joint*>, joints,j)
      {
        (*j)->update();
      }
    }
  /*  if (this->sim_step%6000==0)
    {
      printf("timeSteps   = %li\n",this->sim_step);
      printf("time in min = %f\n",((float)this->sim_step)/100/60);
      printf("(MI0+MI1)/2 = %f\n",(mic->getMI(0)+mic->getMI(1))/2);
      printf("Entropy     = %f\n",trackableEntropySLOW->getValue());
    }*/

/*    if  ((this->convTest0->getValue()==1.0)&&(this->convTest1->getValue()==1.0) &&
           (this->convTest2->getValue()==1.0)&&(this->convTest3->getValue()==1.0) &&
              (this->convTest4->getValue()==1.0)&&(this->convTest5->getValue()==1.0))*/
    // 600.000 = 100min
    // we take 1440min = 8.640.000 = 24h
    //if (this->sim_step==250000)
//    if (this->sim_step==1000)
//if(false)
    //if (this->sim_step==30000)
    {
      //simulation_time_reached=true;
      /*    printf("\nConvergence reached!\n");
      printf("timeSteps   = %li\n",this->sim_step);
      printf("time in min = %f\n",((float)this->sim_step)/100/60);
      printf("(MI0+MI1)/2 = %f\n",(mic->getMI(0)+mic->getMI(1))/2);
      printf("Entropy     = %f\n",trackableEntropySLOW->getValue());      */
    }
  }

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (down)
    { // only when key is pressed, not when released
      switch ( (char) key )
      {
      case 'e':
        /*   printf("timeSteps   = %li\n",this->sim_step);
        printf("time in min = %f\n",((float)this->sim_step)/100/60);
        printf("(MI0+MI1)/2 = %f\n",(mic->getMI(0)+mic->getMI(1))/2);
        printf("Entropy     = %f\n",trackableEntropySLOW->getValue());        */
        return true;
        break;
      default:
        return false;
        break;
      }
    }
    return false;
  }

};

void runSim(double cinit, int runs, int argc, char **argv,double binit=0.0)
{
  double sum = 0.0;
  double misum=0.0;
  double h_xsum=0.0;
  double h_yxsum=0.0;
  double avg = -1.0;
  double mi = -1.0;
  double h_x = -1.0;
  double h_yx = -1.0;
  for (int i=0;i<runs;i++)
  {
    std::cout << "run number " << (i+1) << "..." << std::endl;
    ThisSim* sim;
    sim = new ThisSim(cinit,binit);
    sim->run(argc,argv);
    double val = sim->trackableEntropySLOW->getValue();
    if (val>avg)
      avg=val;
    val = (sim->mic->getMI(0) + sim->mic->getMI(1))/2.0;
    if (val>mi)
      mi=val;
    val = (sim->mic->getH_x(0) + sim->mic->getH_x(1))/2.0;
    if (val>h_x)
      h_x=val;
    val = (sim->mic->getH_yx(0) + sim->mic->getH_yx(1))/2.0;
    if (val>h_yx)
      h_yx=val;
    sum+= sim->trackableEntropySLOW->getValue();
    misum += (sim->mic->getMI(0) + sim->mic->getMI(1))/2.0;
    h_xsum += (sim->mic->getH_x(0) + sim->mic->getH_x(1))/2.0;
    h_yxsum += (sim->mic->getH_yx(0) + sim->mic->getH_yx(1))/2.0;
    delete(sim);
  }
  double loc_avg = sum / ((double)runs);
  double mi_avg = misum / ((double)runs);
  double h_x_avg = h_xsum / ((double)runs);
  double h_yx_avg = h_yxsum / ((double)runs);
  FILE* file;
  char filename[256];
  sprintf(filename, "ent_%f_%f_C.log",cinit,binit);
  file = fopen(filename,"a");
  fprintf(file,"%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",cinit, binit,avg,mi,h_x,h_yx,loc_avg,mi_avg,h_x_avg,h_yx_avg);
  fflush(file);

}

int main (int argc, char **argv)
{
  double startx=0.0;
  double endx=1.3;
  double starty=-1.5;
  double endy=1.5;
  double stepSizex=0.2;
  double stepSizey=0.2;
  int numberSteps = (int)(((endx-startx)/stepSizex+1)*((endy-starty)/stepSizey+1));
  // check for -first value
  int index = Simulation::contains(argv, argc, "-first");
  if(index &&  (argc > index))
  {
    int firstStep=atoi(argv[index]);
    int lastStep=0;
    int stepId = 0;
    int stepInterval=1;
    index = Simulation::contains(argv, argc, "-last");
    if(index &&  (argc > index))
      lastStep=atoi(argv[index]);
    index =  Simulation::contains(argv, argc, "-step");
    if(index &&  (argc > index))
      stepInterval=atoi(argv[index]);
    index =  Simulation::contains(argv, argc, "-id");
    if(index &&  (argc > index))
      stepId=atoi(argv[index]);
    // -0.1 <= cnondiag <= 0.3
    // 0.9 <= cdiag <= 1.8 normally
    // cdiag=0.9 eq. id=90; cdiag=1.5 eq. id=150 etc.
    // cnondiag uses same stepsize, but is restricted to -0.1 to 0.3
    int steps = (lastStep-firstStep)/stepInterval +1;
    // now calculate the real stepSize for C

    // quadratic version, create landscape
//    double stepSize = sqrt(1/((double)(3*steps)));
//    double cdiag = 0.8 + (((stepId-firstStep)/stepInterval) % ((int)(0.5/stepSize))) *stepSize*8.7732;
//    double cnondiag = -3.1415 + ((double)((int)(((stepId-firstStep)/stepInterval) / (0.5/stepSize)))) * stepSize*9.6341;
    // linear version, cnondiag=0
    // double stepSize = 1/((double)steps);
    // double cnondiag = 0.0;
    // double cdiag = 0.8 + ((double)(stepId-firstStep)) * stepSize;

/*    std::cout << "firstStep    = " << firstStep << std::endl;
    std::cout << "lastStep     = " << lastStep << std::endl;
    std::cout << "stepId       = " << stepId << std::endl;
    std::cout << "stepInterval = " << stepInterval << std::endl;
    std::cout << "steps        = " << steps << std::endl;
    std::cout << "stepSize     = " << stepSize << std::endl;
    std::cout << "cdiag        = " << cdiag << std::endl;
    std::cout << "cnondiag     = " << cnondiag << std::endl;
    runSim(cdiag,5,argc,argv,cnondiag);*/


    if ((numberSteps!=steps) || (stepInterval!=1)) {
        if (numberSteps!=steps)
            std::cerr << "wrong number of steps! For the choosen parameters please run with " << numberSteps << " steps!" << std::endl;
        if (stepInterval!=1)
            std::cerr << "wrong number of stepInterval! or the choosen parameters please run with -step 1!" << std::endl;
    } else {
        double x = startx + ((stepId-firstStep) % ((int)((endx-startx)/stepSizex+1)))*stepSizex;
        double y = starty + ((stepId-firstStep) / ((int)((endx-startx)/stepSizex+1)))*stepSizey;
        std::cout << "firstStep    = " << firstStep << std::endl;
        std::cout << "lastStep     = " << lastStep << std::endl;
        std::cout << "stepId       = " << stepId << std::endl;
        std::cout << "steps        = " << steps << std::endl;
        std::cout << "stepSizex     = " << stepSizex << std::endl;
        std::cout << "stepSizey     = " << stepSizey << std::endl;
        std::cout << "cdiag        = " << x << std::endl;
        std::cout << "bnondiag     = " << y << std::endl;
        runSim(x,3,argc,argv,y);

    }
  } else if ( Simulation::contains(argv, argc, "-loop") && (argc >  Simulation::contains(argv, argc, "-last"))){
    // then loop over all cinit and cnondiag values:
    std::cout << "Running now " << numberSteps << " steps, be patient :)" << std::endl;
    for (int i=1;i<=numberSteps;i++) {
        double x = startx + ((i-1) % ((int)((endx-startx)/stepSizex+1)))*stepSizex;
        double y = starty + ((i-1) / ((int)((endx-startx)/stepSizex+1)))*stepSizey;
      std::cout << "---cdiag = " << x << std::endl;
        std::cout << "bnondiag = " << y << std::endl;
        runSim(x,1,argc,argv,y);
    }
  }
  else
  {
    // check for runs value
    int runs=1;
    int index =  Simulation::contains(argv, argc, "-runs");
    if(index &&  (argc > index))
      runs = atoi(argv[index]);
    // check for cinit value
    index =  Simulation::contains(argv, argc, "-cinit");
    double cinit=1.0;
    if (index && (argc > index)) {
      cinit = atof(argv[index]);
      // check for cnondiag value
      index =  Simulation::contains(argv, argc, "-bnondiag");
      double cnondiag=0.2;
      if (index && (argc > index))
        cnondiag=atof(argv[index]);
      for (int i=0;i<runs;i++)
      {
        std::cout << "run number " << (i+1) << "..." << std::endl;
        runSim(cinit,runs,argc,argv,cnondiag);
      }
    } else
    {
      // double fineadjustment=.1;
      /*    for (double cinit=0.0;cinit<0.95;cinit+=fineadjustment)
            runSim(cinit,runs,argc,argv);
          for (double cinit=0.95;cinit<1.15;cinit+=fineadjustment/2.0)
            runSim(cinit,runs,argc,argv);
          for (double cinit=1.15;cinit<=1.3;cinit+=fineadjustment)
            runSim(cinit,runs,argc,argv);
      for (double cinit=1.3;cinit<=2.0;cinit+=fineadjustment)
        runSim(cinit,runs,argc,argv);*/
      runSim(0.55,1,argc,argv,0.0);
    }
  }
}
