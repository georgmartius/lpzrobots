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
 *   Revision 1.9  2011-10-14 09:36:18  martius
 *   snakes have no frictionGround parameter anymore, since it was not used,
 *    use the substances now
 *
 *   Revision 1.8  2011/06/03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.7  2011/03/19 08:47:55  guettler
 *   - unique names are generated even for e.g. spheres, snakes
 *
 *   Revision 1.6  2010/01/26 09:58:15  martius
 *   changed a lot parameter
 *
 *   Revision 1.5  2009/03/25 15:44:23  guettler
 *   ParallelSplitShadowMap: corrected light direction (using directional light), complete ground is now shadowed
 *
 *   Revision 1.4  2008/05/01 22:03:56  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.3  2007/01/26 12:07:09  martius
 *   orientationsensor added
 *
 *   Revision 1.2  2006/09/21 22:11:33  martius
 *   make opt fixed
 *
 *   Revision 1.1  2006/09/20 12:56:40  martius
 *   *** empty log message ***
 *
 *
 ***************************************************************************/
#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>
#include <ode_robots/terrainground.h>

#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/passivecapsule.h>

#include <selforg/invertnchannelcontroller.h>
#include <selforg/dercontroller.h>
#include <selforg/invertmotornstep.h>
#include <selforg/invertmotorspace.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

#include <osg/Light>
#include <osg/LightSource>


#include <ode_robots/hurlingsnake.h>
#include <ode_robots/schlangeservo2.h>
#include <ode_robots/caterpillar.h>
#include <ode_robots/nimm2.h>
#include <ode_robots/nimm4.h>
#include <ode_robots/sphererobot3masses.h>
#include <ode_robots/plattfussschlange.h>
#include <ode_robots/axisorientationsensor.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace osg;

class ThisSim : public Simulation {
public:

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(-19.15, 13.9, 6.9),  Pos(-126.1, -17.6, 0));

    // initialization
    // - set noise to 0.01
    global.odeConfig.setParam("noise",0.05);
    global.odeConfig.setParam("controlinterval",4);
    global.odeConfig.setParam("simstepsize",0.005);
    global.odeConfig.setParam("gravity",-6);
    global.odeConfig.setParam("realtimefactor",1);

    int numSnakeLong=0;
    int numNimm4=1;
    int numHurling=0;
    int numSphere=3;
    int numPlattfuss=0;
    int numSnake=0;

    double height = .3;

    OdeHandle groundHandle(odeHandle);
    groundHandle.substance.toPlastic(1.5);
    Playground* playground =
      new Playground(groundHandle, osgHandle,osg::Vec3(20, 0.2, height+1.5));
    playground->setColor(Color(0.88f,0.4f,0.26f,0.2f));
    playground->setTexture("Images/really_white.rgb");
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground);

    TerrainGround* terrainground =
      new TerrainGround(groundHandle, osgHandle.changeColor(Color(1.0f,1.0,0.0/*196.0/255.0,41.0/255.0*/)),
//                        "terrains/macrospheresTex_256.ppm",
                        "terrains/zoo_landscape1.ppm",
//                        "terrains/terrain_bumpInDip128.ppm",
                        //"",
//                        "Images/dusty.rgb",
                        "terrains/zoo_landscape_texture.ppm",
                        20, 20, height, OSGHeightField::Red);
    terrainground->setPose(osg::Matrix::translate(0, 0, 0.1));
    global.obstacles.push_back(terrainground);

    InvertMotorNStepConf invertnconf = InvertMotorNStep::getDefaultConf();


    for(int i=0; i<3; i++){
      PassiveSphere* s =
        new PassiveSphere(odeHandle,
                          osgHandle.changeColor(Color(184 / 255.0, 233 / 255.0, 237 / 255.0)),
                          0.3);
      s->setPosition(Pos(-1 , i*2-2, height+0.05));
      s->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s);
    }

//     for(int i=0; i<5; i++){
//       PassiveBox* b =
//         new PassiveBox(odeHandle,
//                           osgHandle, osg::Vec3(0.2+i*0.1,0.2+i*0.1,0.2+i*0.1));
//       b->setPosition(Pos(i*0.5-5, i*0.5, height));
//       b->setColor(Color(1.0f,0.2f,0.2f,0.5f));
//       b->setTexture("Images/light_chess.rgb");
//       global.obstacles.push_back(b);
//     }

//     for(int i=0; i<5; i++){
//       PassiveCapsule* c =
//         new PassiveCapsule(odeHandle, osgHandle, 0.2f, 0.3f, 0.3f);
//       c->setPosition(Pos(i-1, -i, height));
//       c->setColor(Color(0.2f,0.2f,1.0f,0.5f));
//       c->setTexture("Images/light_chess.rgb");
//       global.obstacles.push_back(c);
//     }

    OdeAgent* agent;
    AbstractWiring* wiring;
    OdeRobot* robot;
    AbstractController *controller;

//     //******* R A U P E  *********/
//     CaterPillar* myCaterPillar;
//     CaterPillarConf myCaterPillarConf = DefaultCaterPillar::getDefaultConf();
//     myCaterPillarConf.segmNumber=3;
//     myCaterPillarConf.jointLimit=M_PI/3;
//     myCaterPillarConf.motorPower=0.2;
//     myCaterPillarConf.frictionGround=0.01;
//     myCaterPillarConf.frictionJoint=0.01;
//     myCaterPillar =
//       new CaterPillar ( odeHandle, osgHandle.changeColor(Color(1.0f,0.0,0.0)),
//                         myCaterPillarConf, "Raupe");
//     ((OdeRobot*) myCaterPillar)->place(Pos(-5,-5,height));

//      invertnconf.cInit=2.0;
//      controller = new InvertMotorSpace(15);
//     wiring = new One2OneWiring(new ColorUniformNoise(0.1));
//     agent = new OdeAgent( global, plotoptions );
//     agent->init(controller, myCaterPillar, wiring);
//     global.agents.push_back(agent);
//     global.configs.push_back(controller);
//     global.configs.push_back(myCaterPillar);
//     myCaterPillar->setParam("gamma",/*gb");
//     global.obstacles.push_back(s)0.0000*/ 0.0);


    //******* S C H L A N G E  (Long)  *********/
    for(int r=0; r<numSnakeLong;  r++){
      SchlangeServo2* snake;
      SchlangeConf snakeConf = SchlangeServo2::getDefaultConf();
      snakeConf.segmNumber=4;

      snake = new SchlangeServo2 ( odeHandle, osgHandle, snakeConf, "SchlangeLong" );
      ((OdeRobot*) snake)->place(Pos(4,4+3*r,height));
      controller = new InvertMotorNStep(invertnconf);
      wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      agent = new OdeAgent( std::list<PlotOption>() );
      agent->init(controller, snake, wiring);
      global.agents.push_back(agent);
      global.configs.push_back(controller);
      global.configs.push_back(snake);
    }

//     //******* N I M M  2 *********/
//     Nimm2Conf nimm2conf = Nimm2::getDefaultConf();
//     nimm2conf.size = 1.6;
//     for(int r=0; r < 1; r++) {
//       robot = new Nimm2(odeHandle, osgHandle, nimm2conf, "Nimm2_" + std::itos(r));
//       robot->place(Pos ((r-1)*5, 5, height));
//       //    controller = new InvertMotorNStep(10);
//       controller = new InvertMotorSpace(15);
//       controller->setParam("s4avg",10);
//       //    controller->setParam("factorB",0); // not needed here and it does some harm on the behaviour
//       wiring = new One2OneWiring(new ColorUniformNoise(0.1));
//       agent = new OdeAgent( std::list<PlotOption>() );
//       agent->init(controller, robot, wiring);
//       global.configs.push_back(controller);
//       global.agents.push_back(agent);
//     }

    //******* N I M M  4 *********/
    for(int r=0; r < numNimm4; r++) {
      robot = new Nimm4(odeHandle, osgHandle, "Nimm4_" + std::itos(r));
      robot->place(Pos((r-1)*5, -3, height+0.05));
      InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
      cc.cInit=1.01;
      controller = new InvertMotorNStep(cc);
      controller->setParam("s4avg",3);
      controller->setParam("factorB",0.1); // not needed here and it does some harm on the behaviour
      wiring = new One2OneWiring(new ColorUniformNoise(0.1));
      agent = new OdeAgent(global); //  std::list<PlotOption>()
      agent->init(controller, robot, wiring);
      global.agents.push_back(agent);
    }

    //****** H U R L I N G **********/
    InvertMotorNStepConf invertnconf2 = InvertMotorNStep::getDefaultConf();
    for(int r=0; r < numHurling; r++) {
      HurlingSnake* snake;
      Color c;
      if (r==0) c=Color(0.8, 0.8, 0);
      if (r==1) c=Color(0,   0.8, 0);
      snake = new HurlingSnake(odeHandle, osgHandle.changeColor(c), "HurlingSnake_" + std::itos(r));
      ((OdeRobot*) snake)->place(Pos(r*5-4,-6, height+0.2));
      invertnconf2.cInit=1.5;
      controller = new InvertMotorNStep(invertnconf2);
      controller->setParam("steps", 2);
      controller->setParam("epsA", 0.15);
      controller->setParam("epsC", 0.04);
      controller->setParam("adaptrate",  0.000); //0.001);
      controller->setParam("nomupdate",  0.000); //0.001);
      controller->setParam("factorB", 0);

      // deriveconf = DerivativeWiring::getDefaultConf();
      //     deriveconf.blindMotorSets=0;
      //     deriveconf.useId = true;
      //     deriveconf.useFirstD = true;
      //     deriveconf.derivativeScale = 50;
      //     wiring = new DerivativeWiring(deriveconf, new ColorUniformNoise(0.1));
      wiring = new One2OneWiring(new ColorUniformNoise(0.05));
      agent = new OdeAgent( global, plotoptions );
      agent->init(controller, snake, wiring);
                               global.configs.push_back(controller);
                               global.agents.push_back(agent);
    }

    //****** S P H E R E **********/
    for(int r=0; r < numSphere; r++) {
      Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();
      conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
      Sphererobot3Masses* sphere1 =
        new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(r!=1,r==4,r==1)),
                                 conf, "Sphere_" + std::itos(r), 0.2);
      ((OdeRobot*)sphere1)->place ( Pos( -2.5*r , 0 , height+0.05));
      controller = new InvertMotorSpace(15);
      One2OneWiring* wiring2 = new One2OneWiring ( new ColorUniformNoise() );
      agent = new OdeAgent ( r==4? plotoptions : std::list<PlotOption>() );
      agent->init ( controller , sphere1 , wiring2 );
      global.agents.push_back ( agent );
      global.configs.push_back ( controller );
    }

    //creation of flatfoot  snakes
    //****** PLATTFUSS **********/
    for(int i=0; i<numPlattfuss; i++){

      //****************/
      SchlangeConf conf = Schlange::getDefaultConf();
      conf.segmMass   = .2;
      conf.segmLength=.4;
      conf.segmDia=.1;
      conf.motorPower=.3;
      conf.segmNumber = 5+2*i;//-i/2;
      // conf.jointLimit=conf.jointLimit*3;
      conf.jointLimit=conf.jointLimit*2.0;
      conf.frictionJoint=0.1;
      PlattfussSchlange* schlange1;
      if (i==0) {
        schlange1 =
          //new SchlangeServo2 ( odeHandle, osgHandle.changeColor(Color(0.8, 0.3, 0.5)),
          new PlattfussSchlange ( odeHandle, osgHandle.changeColor(Color(1.0, 1.0, 1.0)),
                                  conf, "S1");
        schlange1->setHeadColor(Color(1.0,0,0));
      } else {
        schlange1 =
          //new SchlangeServo2 ( odeHandle, osgHandle.changeColor(Color(0.8, 0.3, 0.5)),
          new PlattfussSchlange ( odeHandle, osgHandle.changeColor(Color(0.8, 0.4, .3)),
                                  conf, "S2");
        schlange1->setHeadColor(Color(0,1.0,0));
      }
      schlange1->setTexture("Images/whitemetal_farbig_small.rgb");

      //Positionieren und rotieren
      schlange1->place(osg::Matrix::rotate(M_PI/2, 0, 1, 0)*
                       // osg::Matrix::translate(-.7+0.7*i,0,(i+1)*(.2+conf.segmNumber)/2.0/*+2*/));
                       osg::Matrix::translate(-7+0.7*i,0,(i+1)*(2+conf.segmNumber)/2.0/*+2*/));
      DerControllerConf cc = DerController::getDefaultConf();
      cc.cInit=1.15;
      AbstractController *controller = new DerController(cc);
      DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
      c.useId = false;
      c.useFirstD = true;
      DerivativeWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise(0.1) );

      OdeAgent* agent = new OdeAgent();
      agent->init(controller, schlange1, wiring);
      global.agents.push_back(agent);
      global.configs.push_back(controller);
      global.configs.push_back(schlange1);


      controller->setParam("steps",1);
      controller->setParam("epsC",0.1);
      controller->setParam("epsA",0.1);
      controller->setParam("adaptrate",0.0);//0.005);
      controller->setParam("rootE",3);
      controller->setParam("logaE",0);

      // controller->setParam("desens",0.0);
      controller->setParam("s4delay",1.0);
      controller->setParam("s4avg",1.0);

      controller->setParam("factorB",0.0);
      controller->setParam("noiseB",0.0);

      controller->setParam("frictionjoint",0.01);
      controller->setParam("teacher", 0.0);

    }//creation of snakes End


    //****** SNAKES **********/
    //creation of normal   snakes
    for(int i=0; i<numSnake; i++){

      //****************/
      SchlangeConf conf = Schlange::getDefaultConf();
      conf.segmMass   = .2;
      conf.segmLength=.8;
      conf.segmDia=.2;
      conf.motorPower=3;
      conf.segmNumber = 5+2*i;//-i/2;
      // conf.jointLimit=conf.jointLimit*3;
      conf.jointLimit=conf.jointLimit*2.0;
      conf.frictionJoint=0.1;
      //PlattfussSchlange* schlange1;
      SchlangeServo2* schlange1;
      if (i==0) {
        schlange1 =
          new SchlangeServo2 ( odeHandle, osgHandle.changeColor(Color(0.8, 0.3, 0.5)),
                               //  new PlattfussSchlange ( odeHandle, osgHandle.changeColor(Color(1.0, 1.0, 1.0)),
                               conf, "S1_" + std::itos(i));
      } else {
        schlange1 =
          new SchlangeServo2 ( odeHandle, osgHandle.changeColor(Color(0.8, 0.3, 0.5)),
                               // new PlattfussSchlange ( odeHandle, osgHandle.changeColor(Color(0.8, 0.4, .3)),
                               conf, "S2_" + std::itos(i));
      }
      //Positionieren und rotieren
      schlange1->place(osg::Matrix::rotate(M_PI/2, 0, 1, 0)*
                       // osg::Matrix::translate(-.7+0.7*i,0,(i+1)*(.2+conf.segmNumber)/2.0/*+2*/));
                       osg::Matrix::translate(-.7+0.7*i,0,(i+1)*(2+conf.segmNumber)/2.0/*+2*/));
      schlange1->setTexture("Images/whitemetal_farbig_small.rgb");
      if (i==0) {
        schlange1->setHeadColor(Color(1.0,0,0));
      } else {
        schlange1->setHeadColor(Color(0,1.0,0));
      }


      //      AbstractController *controller = new InvertMotorNStep();
      DerControllerConf cc = DerController::getDefaultConf();
      cc.cInit=1.2;
      AbstractController *controller = new DerController(cc);
      //AbstractController *controller = new SineController();

      //  AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.05)); //Only this line for one2Onewiring
      DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
      c.useId = false;
      c.useFirstD = true;
      DerivativeWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise() );

      OdeAgent* agent = new OdeAgent();
      agent->init(controller, schlange1, wiring);
      global.agents.push_back(agent);
      global.configs.push_back(controller);
      global.configs.push_back(schlange1);


      controller->setParam("steps",1);
      controller->setParam("epsC",0.1);
      controller->setParam("epsA",0.1);
      controller->setParam("adaptrate",0.0);//0.005);
      controller->setParam("rootE",3);
      controller->setParam("logaE",0);

      // controller->setParam("desens",0.0);
      controller->setParam("s4delay",1.0);
      controller->setParam("s4avg",1.0);

      controller->setParam("factorB",0.0);
      controller->setParam("noiseB",0.0);

      controller->setParam("frictionjoint",0.01);
      controller->setParam("teacher", 0.0);

    }//creation of snakes End





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

  virtual osg::LightSource* makeLights(osg::StateSet* stateset)
  {
    // create a spot light.
    Light* light_0 = new Light;
    light_0->setLightNum(0);
    light_0->setPosition(Vec4(40.0f, 40.0f, 50.0f, 1.0f));
    // note that the blue component doesn't work!!! (bug in OSG???)
    //   Really? It works with me! (Georg)
    light_0->setAmbient(Vec4(0.5f, 0.5f, 0.5f, 1.0f));
    light_0->setDiffuse(Vec4(0.8f, 0.8f, 0.8f, 1.0f));
    //    light_0->setDirection(Vec3(-1.0f, -1.0f, -1.2f));
    light_0->setSpecular(Vec4(1.0f, 0.9f, 0.8f, 1.0f));

    LightSource* light_source_0 = new LightSource;
    light_source_0->setLight(light_0);
    light_source_0->setLocalStateSetModes(StateAttribute::ON);
    light_source_0->setStateSetModes(*stateset, StateAttribute::ON);

    return light_source_0;
  }




};

int main (int argc, char **argv)
{
  ThisSim sim;
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}

