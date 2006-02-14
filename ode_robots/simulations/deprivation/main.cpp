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
 *   Revision 1.1.2.1  2006-02-14 10:32:28  martius
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

#include "simulation.h"

#include "odeagent.h"
#include <selforg/deprivation.h>
#include <selforg/one2onewiring.h>

#include "nimm2.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace matrix;

// plotoptions is a list of possible online output, 
// if the list is empty no online gnuplot windows and no logging to file occurs.
// The list is modified with commandline options, see main() at the bottom of this file
list<PlotOption> plotoptions;

Deprivation* controller;
int zeit = 0;

Matrix straightMotor(const Matrix _dont_care){  
  Matrix y(_dont_care.getM(),1);
  for(int i=0; i< y.getM(); i++){
    y.val(i,0) = sin(zeit/100.0)*0.9;
  }
  zeit++;
  return y;
}

Matrix turnMotor(const Matrix _dont_care){  
  Matrix y(_dont_care.getM(),1);
  for(int i=0; i< y.getM(); i++){
    y.val(i,0) = pow(-1,i)*sin(zeit/100.0)*0.9;
  }
  zeit++;
  return y;
}

Matrix sinMotor(const Matrix _dont_care){  
  Matrix y(_dont_care.getM(),1);
  y.val(0,0) = sin(zeit/10.0)*0.9;
  y.val(1,0) = cos(zeit/10.0)*0.9;
  for(int i=2; i< y.getM(); i++){
    y.val(i,0) = sin(zeit/10.0)*0.9;
  }
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
    OdeRobot* vehicle = new Nimm2(odeHandle, osgHandle, c);
    //OdeRobot* vehicle = new Nimm4(odeHandle, osgHandle);
    vehicle->place(Pos(0,0,0.0));

    // create pointer to controller
    // push controller in global list of configurables
    //  AbstractController *controller = new InvertNChannelController(10);      
    InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();    
    cc.cInit=1.0;
    controller = new Deprivation(sinMotor, cc);  
    controller->setExternalControlMode(true);
    controller->setParam("adaptrate", 0.000);
    //    controller->setParam("nomupdate", 0.0005);
    controller->setParam("epsC", 0.1);
    controller->setParam("epsA", 0.05);
    controller->setParam("rootE", 1);
    controller->setParam("steps", 2);
    controller->setParam("s4avg", 1);

    //    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    One2OneWiring* wiring = new One2OneWiring(new WhiteUniformNoise());
    OdeAgent* agent = new OdeAgent(plotoptions);
    agent->init(controller, vehicle, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(controller);
      
    showParams(global.configs);
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
	controller->store("test") && printf("Controller stored\n");
	handled = true; break;	
      case 'l' :
	controller->restore("test") && printf("Controller loaded\n");
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

// print command line options
void printUsage(const char* progname){
  printf("Usage: %s [-g] [-f]\n\t-g\tuse guilogger\n\t-f\tuse guilogger with logfile\n", progname);
}

int main (int argc, char **argv)
{ 
  // start with online windows (default: start without plotting and logging)
  if(contains(argv, argc, "-g")) plotoptions.push_back(PlotOption(GuiLogger));
  
  // start with online windows and logging to file
  if(contains(argv, argc, "-f")) plotoptions.push_back(PlotOption(GuiLogger_File, Controller, 10));

  // 
  if(contains(argv, argc, "-n")) plotoptions.push_back(PlotOption(NeuronViz, Controller, 10));
  
  // display help
  if(contains(argv, argc, "-h")) printUsage(argv[0]);

  ThisSim sim;
  return sim.run(argc, argv) ? 0 :  1;
}

 
 
