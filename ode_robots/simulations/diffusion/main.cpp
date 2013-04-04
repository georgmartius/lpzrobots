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
 *   Revision 1.4  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.3  2009/10/23 13:07:47  martius
 *   *** empty log message ***
 *
 *   Revision 1.2  2008/05/01 22:03:54  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.1  2007/06/21 16:24:56  martius
 *   another multiagent simulation with nimm2
 *
 *
 ***************************************************************************/

#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/passivesphere.h>

#include <selforg/sinecontroller.h>
#include <selforg/invertmotornstep.h>
#include <selforg/invertmotorspace.h>
#include <selforg/invertnchannelcontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/stl_adds.h>

#include <ode_robots/nimm2.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


class ThisSim : public Simulation {
public:
  int numx;
  int numy;
  double regionsize;

  ThisSim(int numx, int numy, double regionsize)
    : numx(numx), numy(numy), regionsize(regionsize)
  {
  }

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));

    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.noise=0.05;
    //    global.odeConfig.setParam("gravity", 0);
    global.odeConfig.setParam("controlinterval", 1);
    global.odeConfig.setParam("realtimefactor",0);
    global.odeConfig.setParam("simstepsize",0.04);

    if(regionsize==-1){// automatic mode
      regionsize=sqrt(numx*numy);
    }
    if(regionsize >0){
      // use Playground as boundary:
      OctaPlayground* playground = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(regionsize, 0.2, 0.5), 36);
      playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
      global.obstacles.push_back(playground);
    }
    // for(int i=0; i<50; i++){
//       PassiveSphere* s = new PassiveSphere(odeHandle, osgHandle.changeColor(Color(0.0,1.0,0.0)), 0.5);
//       s->setPosition(osg::Vec3(-4+(i/10),-4+(i%10),1));
//       global.obstacles.push_back(s);
//     }

    OdeRobot* nimm2;
    AbstractController* contrl;
    AbstractWiring* wiring;
    OdeAgent* agent;

    for (int j=0; j<numx; j++){
      for (int i=0; i<numy; i++){
        //      nimm2 = new Nimm2(odeHandle);
        Nimm2Conf conf = Nimm2::getDefaultConf();
//         conf.speed=20;
//         conf.force=3.0;
//         conf.bumper=false;
//         conf.cigarMode=false;
        wiring = new One2OneWiring(new ColorUniformNoise(0.1));
        contrl = new InvertNChannelController(10);

        //         if ((i==0) && (j==0)) {
        //           agent = new OdeAgent(global);

        //           nimm2 = new Nimm2(odeHandle, osgHandle, conf, "Nimm2Yellow");
        //           nimm2->setColor(Color(1.0,1.0,0));
        //           global.configs.push_back(contrl);
        //           agent->init(contrl, nimm2, wiring);
        //           //    controller->setParam("nomupdate", 0.0005);
        //         } else {

        agent = new OdeAgent(global,NoPlot);
        nimm2 = new Nimm2(odeHandle, osgHandle, conf, "Nimm2_" + std::itos(i) + "_" + std::itos(j));
        agent->init(contrl, nimm2, wiring);

        agent->setTrackOptions(TrackRobot(true,true,false,false,(std::itos(numx*numy) + "_" + std::itos((int)regionsize)).c_str(), 1));
        nimm2->place(Pos( (j-numx/2)*1.25,(i-numy/2)*1.25,0));
        global.agents.push_back(agent);
      }
    }


  }

};

int main (int argc, char **argv)
{
  if(argc<3){
    fprintf(stderr,"Usage: %s num_x num_y [size]\n  size: use -1 for automatic size\n",argv[0]);
    exit(1);
  }
  double s=-1;
  if(argc>3){
    s=atof(argv[3]);
  }
  ThisSim sim(atoi(argv[1]), atoi(argv[2]),s);
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}

