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
 *   Revision 1.5  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.4  2008/11/14 11:23:05  martius
 *   added centered Servos! This is useful for highly nonequal min max values
 *   skeleton has now also a joint in the back
 *
 *   Revision 1.3  2008/05/01 22:03:54  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.2  2006/07/14 12:23:45  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.5  2006/05/15 13:11:29  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.1.2.4  2006/03/29 15:10:22  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.3  2006/02/24 14:42:39  martius
 *   keys
 *
 *   Revision 1.1.2.2  2006/02/20 10:50:20  martius
 *   pause, random, windowsize, Ctrl-keys
 *
 *   Revision 1.1.2.1  2006/02/14 10:32:28  martius
 *   test env for model deprivation
 *
 *   Revision 1.1.2.6  2006/02/08 16:15:27  martius
 *   teaching via keys
 *
 *   Revision 1.1.2.5  2006/01/31 16:45:50  martius
 *   neuronviz output
 *
 *   Revision 1.1.2.4  2006/01/18 16:47:06  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.3  2006/01/17 17:02:47  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.2  2006/01/13 12:33:16  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.1  2006/01/13 12:24:06  martius
 *   env for external teaching input to the controller
 *
 *
 ***************************************************************************/

#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include "deprivation.h"
#include <selforg/one2onewiring.h>

#include <ode_robots/nimm2.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace matrix;

Deprivation* controller;
int zeit = 0;

Matrix straightMotor(const Matrix& _dont_care){
  Matrix y(_dont_care.getM(),1);
  for(unsigned int i=0; i< y.getM(); i++){
    y.val(i,0) = sin(zeit/100.0)*0.9;
  }
  zeit++;
  return y;
}

void straightController(Matrix& C, Matrix& H ){
  double v = 2.4/(C.getM()*C.getN());
  fprintf(stderr, "pla %g\n", v);
  for(unsigned int i=0; i< C.getM(); i++){
    for(unsigned int j=0; j< C.getN(); j++){
      C.val(i,j) = v;
    }
  }
  for(unsigned int i=0; i< min(C.getN(), C.getM()); i++){
    C.val(i,i) += ((double)rand() / RAND_MAX)*0.01;
  }
  for(unsigned int i=0; i<H.getM(); i++){
    H.val(i,0) = 0;
  }
  controller->setParam("epsC", 0.0005);
}

Matrix turnMotor(const Matrix& _dont_care){
  Matrix y(_dont_care.getM(),1);
  for(unsigned int i=0; i< y.getM(); i++){
    y.val(i,0) = pow(-1.0,i)*sin(zeit/100.0)*0.9;
  }
  zeit++;
  return y;
}

double data[2] = {1,0};
Matrix y(2,1, data);
Matrix sinMotor(const Matrix& y_controller){
  Matrix A(2,2);
  double alpha = M_PI/((cos(zeit/1000.0)+1)*10+1);
  //  if(zeit%100==0) printf("Alpha: %d\n", int(alpha*180/M_PI));
  A.val(0,0) = cos(alpha);
  A.val(1,0) = sin(alpha);
  A.val(0,1) = -sin(alpha);
  A.val(1,1) = cos(alpha);
  y = A*y;
  zeit++;
  return y;
}


class ThisSim : public Simulation {
public:

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(-5.44372, 7.37141, 3.31768),  Pos(-142.211, -21.1623, 0));
    // initialization
    // - set noise to 0.01
    global.odeConfig.setParam("noise", 0.01);
    //    global.odeConfig.setParam("gravity", 0);

    Nimm2Conf c = Nimm2::getDefaultConf();
    OdeRobot* vehicle = new Nimm2(odeHandle, osgHandle, c,"Robot1");
    //OdeRobot* vehicle = new Nimm4(odeHandle, osgHandle);
    vehicle->place(Pos(0,0,0.0));

    // create pointer to controller
    // push controller in global list of configurables
    //  AbstractController *controller = new InvertNChannelController(10);
    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
    cc.cInit=1.0;
    controller = new Deprivation(straightMotor, straightController, cc);
    controller->setParam("adaptrate", 0.005);
    controller->setParam("nomupdate", 0.002);
    controller->setParam("epsC", 0.0005);
    controller->setParam("epsA", 0.01);
    controller->setParam("dampA", 0.01);
    controller->setParam("rootE", 1);
    controller->setParam("steps", 2);
    controller->setParam("s4avg", 1);
    //    controller->setParam("s4delay", 1);

    // One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    One2OneWiring* wiring = new One2OneWiring(new WhiteUniformNoise());
    OdeAgent* agent = new OdeAgent(global);
    agent->init(controller, vehicle, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(controller);
    controller->setExternalControlMode(true);



  }

  //Funktion die eingegebene Befehle/kommandos verarbeitet
  virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (!down) return false;
    bool handled = false;
    switch ( key )
      {
      case 't' :
        controller->setExternalControlMode(!controller->getExternalControlMode());
        printf("Control Mode: %i\n", controller->getExternalControlMode());
        handled = true; break;
      case 's' :
        controller->storeToFile("test") && printf("Controller stored\n");
        handled = true; break;
      case 'l' :
        controller->restoreFromFile("test") && printf("Controller loaded\n");
        handled = true; break;
      }
    fflush(stdout);
    return handled;
  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Deprivation: t","toggle mode (straight moving/controller)");
    au.addKeyboardMouseBinding("Simulation: s","store");
    au.addKeyboardMouseBinding("Simulation: l","load");
  }

};

int main (int argc, char **argv)
{
  ThisSim sim;
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}

