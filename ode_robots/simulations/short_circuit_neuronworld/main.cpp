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
 *   Revision 1.4  2009/12/04 18:51:59  fhesse
 *   invertnchannelcontroller has bias (changeable in constructor) now
 *   neuronworld has linear neuron now (changeable in conf)
 *
 *   Revision 1.3  2009/12/02 10:24:09  fhesse
 *   bias in invertnchannelcontroller added, linear neuron in neuronworld added
 *
 *   Revision 1.2  2009/12/01 13:35:50  fhesse
 *   minor changes
 *
 *   Revision 1.1  2009/09/22 08:21:49  fhesse
 *   world is a schmitt trigger neuron
 *   only 1 DOF so far
 *
 *
 ***************************************************************************/
#include <iostream>
//#include <string>
#include <fstream>
#include  <sstream>
#include <unistd.h>

#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/shortcircuit.h>

#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>
#include <selforg/derivativewiring.h>

//#include <selforg/sinecontroller.h>
//#include <selforg/invertmotornstep.h>
//#include <selforg/invertmotorspace.h>
//#include <selforg/invertnchannelcontroller.h>
#include "invertnchannelcontroller.h"

#include <neuronworld.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

int channels;
int t=0;
double omega = 0.05;

double my_theta_const = -0.45;
double my_gamma = 0.0;
double my_w = 1.0;


SineWhiteNoise* sineNoise;

class ThisSim : public Simulation {

public:

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));

    // initialization
    global.odeConfig.setParam("noise",0.05);
    global.odeConfig.setParam("realtimefactor",1);
    global.odeConfig.setParam("drawinterval", 50);
    global.odeConfig.setParam("realtimefactor", 0);


//    OdeRobot* robot = new ShortCircuit(odeHandle, osgHandle, channels, channels);
      NeuronWorldConf conf = NeuronWorld::getDefaultConf();
      conf.theta_const  = my_theta_const;
      conf.gamma  = my_gamma;
      conf.w  = my_w;
      conf.neuron_type = linear;
      std::ostringstream tmp_name;
      tmp_name<<"ga"<<my_gamma<<"_th"<<my_theta_const<<"_w"<<my_w;
      OdeRobot* robot = new NeuronWorld(odeHandle, osgHandle, channels, channels, conf, tmp_name.str());
      global.configs.push_back(robot);


//     InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
//     cc.cInit=1;
//     cc.cNonDiag=0.5;
//     cc.someInternalParams=false;
//     AbstractController *controller = new InvertMotorNStep(cc);
    //AbstractController *controller = new InvertNChannelController(100);
    AbstractController *controller = new InvertNChannelController(100, /*update_only_1=*/false, /*model_type=*/InvertNChannelController::bias);
    global.configs.push_back(controller);



    controller->setParam("eps",0.1);
    //controller->setParam("eps",0.0);
    controller->setParam("factor_a",0.1);
    controller->setParam("s4avg",1);

    OdeAgent* agent = new OdeAgent(global);
    // sineNoise = new SineWhiteNoise(omega,2,M_PI/2);
    // One2OneWiring* wiring = new One2OneWiring(sineNoise, true);
    One2OneWiring* wiring = new One2OneWiring(new WhiteUniformNoise(), true);
    //    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.05), true);
    //AbstractWiring* wiring = new SelectiveOne2OneWiring(sineNoise, &select_firsthalf);
    // DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
//     c.useId=true;
//     c.useFirstD=false;
//     c.derivativeScale=20;
//     c.blindMotorSets=0;
//     AbstractWiring* wiring = new DerivativeWiring(c, new ColorUniformNoise(0.05));
    agent->init(controller, robot, wiring);
    global.agents.push_back(agent);

  }



  void command(const OdeHandle& odeHandle, GlobalData& global, int key){
    switch (key){
    case '>': omega+=0.05;
      break;
    case '<': omega-=0.05;
      break;
    case '.': omega+=0.005;
      break;
    case ',': omega-=0.005;
      break;
    case 'r': omega=0.05;
      break;
    case 'n': omega=0;
      break;
    }
    fprintf(stderr, "Omega: %g\n", omega);
    sineNoise->setOmega(omega);
  }

  // note: this is the normal signature (look above)
  // add own key handling stuff here, just insert some case values
//   virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
//   {
//     if (down) { // only when key is pressed, not when released
//       switch ( (char) key )
//         {
//         default:
//           return false;
//           break;
//         }
//     }
//     return false;
//   }


};

void printUsage(const char* progname){
  printf("Usage: %s numchannels [-g] [-f]\n\tnumchannels\tnumber of channels\n\
\t-g\t\tuse guilogger\n\t-f\t\tuse guilogger with logfile", progname);
}

int main (int argc, char **argv)
{
  if(argc <= 1){
    printUsage(argv[0]);
    return -1;
  }
  channels = std::max(1,atoi(argv[1]));

  int loopcounter=0;
  int loopmax=1*49*49;

 // system("rm  fft_matrix.dat "); //alte datei loeschen

  for (int g=0;g<1;g++){   // erstmal nur mit gamma=0
    for (int t=0;t<49;t++){
      for (int w=0;w<49;w++){
        my_gamma = g*0.5; // wenn g<3 grenzfall waere: if (my_gamma==1) my_gamma=0.99;
        my_theta_const = t*0.05 - 1.2;
        my_w= w*0.05 - 1.2;
        if ( (w<0.001) && (w>-0.001) ) w=0; // sonst Probleme im Dateinamen  (1.11022e-16)
        ThisSim sim;
        sim.run(argc, argv);

/////////////
// REMOVE !!!
/////////////
return 1;

        //sim.~Simulation();
       loopcounter++;
       std::cout<<std::endl<<"run "<<loopcounter<<" of "<<loopmax<<" finished"<<std::endl<<std::endl;

       // generate name of robot and hence of logfile
       std::ostringstream tmp_name;
       tmp_name<<"ga"<<my_gamma<<"_th"<<my_theta_const<<"_w"<<my_w;

       // Some handling of logfiles
       std::ostringstream sed_command;   // copy infos from head of logfile (first 41 lines) to filename.head
       sed_command<<"sed 41q "<<tmp_name.str()<<".log >"<<tmp_name.str()<<".head";
       //system("sed 41q ga0_th0_w0.log > ga0_th0_w0.head");
       std::string test=sed_command.str();
       system(test.c_str());

       std::ostringstream sed_command2;            // copy dataset 300000-end to filename.dat (first 300000 lines ignored)
       sed_command2<<"sed '1,300000d' "<<tmp_name.str()<<".log >"<<tmp_name.str()<<".dat";
       //system("sed '1,300000d' ga0_th0_w0.log > ga0_th0_w0.dat");
       std::string test2=sed_command2.str();
       system(test2.c_str());


//       // Do mathematica stuff
//       std::ofstream myfile("mathematica_commands.tmp", std::ios::trunc); // trunc -> alten Dateiinhalt loeschen
//       if (myfile.is_open()){
//         myfile << "Print[\"Processing  "<<tmp_name.str()<<".dat\"] \n";
//         myfile << "x = ReadList[\"./"<<tmp_name.str()<<".dat\", Number, RecordLists -> True]; \n";
//         myfile << "F = Abs[Fourier[ x[[1 ;; 50000, 2]] - Mean[x[[1 ;; 50000, 2]]] ] ]^2;\n";
//         myfile << "pos = Position[F[[1;;25000]], Max[F[[1;;25000]]]] [[1,1]]\n";
//         myfile << "out=StringForm[\"``        ``        ``        ``\","<<my_gamma<<","<<my_theta_const<<", "<<my_w<<", pos];\n";
//         myfile << "Print[out]\n";
//         myfile << "Exit[]\n";
//         myfile.close();
//       }
//       system("/usr/nld/mathematica-6.0/Executables/math -nopromt -run \"<<mathematica_commands.tmp\" >ausgabe.tmp ");
//       system("sed '1,3d' ausgabe.tmp >> fft_matrix.dat ");  // nur letzte Zeile (Ergebnis der Berechnung) aus ausgabe.tmp nach fft_matrix.dat kopieren (anhaengen)



//       // gzip .log file to save disk space
//       std::cout<<"zipping "<<tmp_name.str()<<".log\n"<<std::endl;
//       std::ostringstream gzip_command;
//       gzip_command<<"gzip "<<tmp_name.str()<<".log";
//       std::string gzip_str=gzip_command.str();
//       system(gzip_str.c_str());

       // delete .log file to save disk space
       std::cout<<"deleting "<<tmp_name.str()<<".log\n"<<std::endl;
       std::ostringstream rm_command;
       rm_command<<"rm "<<tmp_name.str()<<".log";
       std::string rm_str=rm_command.str();
       system(rm_str.c_str());

      }
    }
  }
  return 1;


  //ThisSim sim;
  //  return sim.run(argc, argv) ? 0 : 1;

}

/*
Mit
invertmotornstep/motorspace
  epsC=epsA=0.6
  noise=0.05
Es scheint wichtig zu sein, dass das Modell mitlernt.

Bei mehreren steps muss epsC runtergedreht werden.

*/

