/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
#include <cassert>

// include simulation environment stuff
#include <ode_robots/simulation.h>

// used passive spheres and boxes
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/passivecapsule.h>

using namespace std;

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


class ThisSim : public Simulation {
public:
  ThisSim(){
    addPaletteFile("testPalette.gpl", true);
    addColorAliasFile("testAliases.txt", true);
  }
  

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    setCameraHomePos(Pos(12.2217, 9.05786, 5.82483),  Pos(137.165, -12.3091, 0));
    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.setParam("noise", 0);
    global.odeConfig.setParam("simstepsize", 0.002); 


    OsgHandle mh(osgHandle);
    //mh.colorSchema()->print(cout);
    // check loading of files
    assert(mh.colorSchema()->existsColor("test1"));
    assert(mh.colorSchema()->existsColor("test2"));
    assert(mh.colorSchema()->existsColor("test3"));
    assert(mh.colorSchema()->existsColor("test4"));

    mh.colorSchema()->color("test1").print(cerr); cerr << endl;
    assert(mh.colorSchema()->color("test1")==Color::rgb255(200,0,0));
    mh.colorSchema()->color("test2").print(cerr); cerr << endl;
    assert(mh.colorSchema()->color("test2")==Color::rgb255(0,200,0));
    mh.colorSchema()->color("test3").print(cerr); cerr << endl;
    assert(mh.colorSchema()->color("test3")==Color::rgb255(0,0,200));
    mh.colorSchema()->color("test4").print(cerr); cerr << endl;
    assert(mh.colorSchema()->color("test4")==Color::rgb255(200,200,0));
    

    Color c;
    // check alias loading
    assert(mh.colorSchema()->color(c,"alias1"));
    assert(mh.colorSchema()->color(c,"alias2",1));
    // check alias semantics
    
    cerr << "Aliases" << endl;
    mh.colorSchema()->color("alias1",0).print(cerr); cerr << endl;
    assert(mh.colorSchema()->color("alias1",0)==Color::rgb255(200,0,0));
    mh.colorSchema()->color("alias1",3).print(cerr); cerr << endl;
    assert(mh.colorSchema()->color("alias1",3)
           == mh.colorSchema()->color("alias1",0));
    mh.colorSchema()->color("alias2",1).print(cerr); cerr << endl;
    assert(mh.colorSchema()->color("alias2",1)==Color::rgb255(0,0,200));    

    mh.colorSchema()->color("alias2",0).print(cerr); cerr << endl;
    assert(mh.colorSchema()->color("alias2",0)==mh.colorSchema()->getDefaultColor());
    
    exit(0);
  };

};


int main (int argc, char **argv)
{ 
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;  
}
 
