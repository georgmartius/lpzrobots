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
 *   Revision 1.7  2007-06-22 14:25:07  martius
 *   *** empty log message ***
 *
 *   Revision 1.6  2007/06/21 16:31:54  martius
 *   *** empty log message ***
 *
 *   Revision 1.5  2007/06/18 08:11:22  martius
 *   nice version with many agents
 *
 *   Revision 1.4  2007/06/14 08:01:45  martius
 *   Pred error modulation by distance to minimum works
 *
 *   Revision 1.3  2007/06/08 15:37:22  martius
 *   random seed into OdeConfig -> logfiles
 *
 *   Revision 1.2  2007/05/23 14:07:34  martius
 *   *** empty log message ***
 *
 *   Revision 1.14  2007/03/26 13:15:51  martius
 *   new makefile with readline support
 *
 *   Revision 1.13  2007/02/23 19:36:42  martius
 *   useSD
 *
 *   Revision 1.12  2007/02/23 15:14:17  martius
 *   *** empty log message ***
 *
 *   Revision 1.11  2007/01/26 12:07:08  martius
 *   orientationsensor added
 *
 *   Revision 1.10  2006/12/21 11:43:05  martius
 *   commenting style for doxygen //< -> ///<
 *   new sensors for spherical robots
 *
 *   Revision 1.9  2006/12/01 16:19:05  martius
 *   barrel in use
 *
 *   Revision 1.8  2006/11/29 09:16:09  martius
 *   modell stuff
 *
 *   Revision 1.7  2006/07/14 12:23:52  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.6.4.6  2006/05/15 13:11:29  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.6.4.5  2006/05/15 12:29:43  robot3
 *   handling of starting the guilogger moved to simulation.cpp
 *   (is in internal simulation routine now)
 *
 *   Revision 1.6.4.4  2006/03/29 15:10:22  martius
 *   *** empty log message ***
 *
 *   Revision 1.6.4.3  2006/02/17 16:47:55  martius
 *   moved to new system
 *
 *   Revision 1.15.4.3  2006/01/12 15:17:39  martius
 *   *** empty log message ***
 *
 *   Revision 1.15.4.2  2006/01/10 20:33:50  martius
 *   moved to osg
 *
 *   Revision 1.15.4.1  2005/11/15 12:30:17  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.15  2005/11/09 14:54:46  fhesse
 *   nchannelcontroller used
 *
 *   Revision 1.14  2005/11/09 13:41:25  martius
 *   GPL'ised
 *
 ***************************************************************************/

#include "simulation.h"

#include "odeagent.h"
#include "octaplayground.h"
#include "playground.h"
#include "passivesphere.h"

#include <selforg/invertmotornstep.h>
#include <selforg/invertmotorspace.h>
#include <selforg/sinecontroller.h>
#include <selforg/ffnncontroller.h>
#include "multisat.h"
#include <selforg/replaycontroller.h>

#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>
#include <selforg/derivativewiring.h>

#include "forcedsphere.h"
#include "sphererobot3masses.h"
#include "barrel2masses.h"
#include "axisorientationsensor.h"
#include "speedsensor.h"
#include "replayrobot.h"


#include <osg/Node>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osg/TexEnv>
#include <osg/TexGen>
#include <osg/Depth>
#include <osg/StateSet>
#include <osg/ClearNode>
#include <osg/Transform>
#include <osg/MatrixTransform>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/ShapeDrawable>
#include <osg/PolygonOffset>
#include <osg/CullFace>
#include <osg/TexGenNode>
using namespace osg;

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


class ThisSim : public Simulation {
public:
  OdeRobot* sphere1;
  AbstractController *controller;
  MultiSat *multisat;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    int num_barrels=0;
    int num_spheres=1;          
    controller=0;
    multisat=0;
    sphere1=0;

    setCameraHomePos(Pos(-0.497163, 11.6358, 3.67419),  Pos(-179.213, -11.6718, 0));
    // initialization
    global.odeConfig.setParam("noise",0.05);
    //  global.odeConfig.setParam("gravity",-10);
    global.odeConfig.setParam("controlinterval",2);
    

    /* * * * BARRELS * * * */
    for(int i=0; i< num_barrels; i++){
      //****************
      Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();  
      conf.pendularrange  = 0.15; 
      conf.motorsensor=false;
      conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection, Sensor::X | Sensor::Y));
      //      conf.addSensor(new SpeedSensor(5, SpeedSensor::Translational, Sensor::X ));
      conf.addSensor(new SpeedSensor(5, SpeedSensor::RotationalRel, Sensor::Z ));
      conf.irAxis1=false;
      conf.irAxis2=false;
      conf.irAxis3=false;
      conf.spheremass   = 0.3; // 1
      sphere1 = new Barrel2Masses ( odeHandle, osgHandle.changeColor(Color(0.0,0.0,1.0)), 
				    conf, "Multi4_4h_Barrel", 0.4); 
      sphere1->place ( osg::Matrix::rotate(M_PI/2, 1,0,0));
      
      InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
      cc.cInit=0.5;
      cc.useSD=true;
      AbstractController* controller = new InvertMotorNStep(cc); // selforg controller
      //AbstractController* controller = new SineController(); 
      //controller = new FFNNController("models/barrel/controller/nonoise.cx1-10.net", 10, true);
      MultiSatConf msc = MultiSat::getDefaultConf();
      msc.controller = controller;
      msc.numContext = 1;
      msc.numHidden = 4;
      msc.numSats = 4;
      multisat = new MultiSat(msc);
      
      controller->setParam("steps", 2);    
      //    controller->setParam("adaptrate", 0.001);    
      controller->setParam("adaptrate", 0.0);    
      controller->setParam("nomupdate", 0.005);    
      //      controller->setParam("epsC", 0.03);    
      //      controller->setParam("epsA", 0.05);    
      controller->setParam("epsC", 0.02);    
      controller->setParam("epsA", 0.03);    
      controller->setParam("rootE", 3);    
      controller->setParam("logaE", 0);    
    
//       DerivativeWiringConf dc = DerivativeWiring::getDefaultConf();
//       dc.useId=true;
//       dc.useFirstD=false;
//       AbstractWiring* wiring = new DerivativeWiring(dc,new ColorUniformNoise());
//      AbstractWiring* wiring = new SelectiveOne2OneWiring(new ColorUniformNoise(), new select_from_to(0,1));
      AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.2));
      //      OdeAgent* agent = new OdeAgent ( PlotOption(File, Robot, 1) );
      OdeAgent* agent = new OdeAgent ( plotoptions );
      agent->addInspectable(controller); // add selforg controller to list of inspectables
      agent->init ( multisat , sphere1 , wiring );
      //  agent->setTrackOptions(TrackRobot(true, false, false, "ZSens_Ring10_11", 50));
      global.agents.push_back ( agent );
      global.configs.push_back ( controller );
      global.configs.push_back ( multisat );
    }


    /* * * * SPHERES * * * */
    for(int i=0; i< num_spheres; i++){
      bool replay=false;
      global.odeConfig.setParam("noise", replay ? 0 : 0.1);
      //****************
      const char* replayfilename="Sphere_reinforce_axis_rot.sel.log";
      Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();  
      conf.pendularrange  = 0.15; 
      conf.motorpowerfactor  = 150;    
      conf.motorsensor=false;
      conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
      //      conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::Axis));
      conf.addSensor(new SpeedSensor(5, SpeedSensor::RotationalRel));

      sphere1 = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(0,0.0,2.0)), 
					 conf, "Multi20_2h_Sphere", 0.3);     
      sphere1->place ( osg::Matrix::rotate(M_PI/2, 1,0,0)); 
      
      if(!replay){
	InvertMotorNStepConf cc = InvertMotorNStep::getDefaultConf();
	//      DerControllerConf cc = DerController::getDefaultConf();
	cc.cInit=1.0;
	//      cc.useSD=true;
	//controller = new DerController(cc);    
	controller = new InvertMotorNStep(cc);    
      }else 
	controller = new ReplayController(replayfilename,true);     

      MultiSatConf msc = MultiSat::getDefaultConf();
      msc.controller = controller;
      msc.numContext = 3;
      msc.numSomPerDim = 3;
      msc.numHidden = 2;
      msc.numSats   = 6;
      msc.eps0      = 0.005;
      msc.deltaMin  = 0.0005;
      msc.tauC      = 2000;
      msc.useDerive=false;
      multisat = new MultiSat(msc);

      //controller = new FFNNController("models/barrel/controller/nonoise.cx1-10.net", 10, true);
      controller->setParam("steps", 1);    
      //    controller->setParam("adaptrate", 0.001);    
      controller->setParam("adaptrate", 0.0);    
      controller->setParam("nomupdate", 0.005);    
      controller->setParam("epsC", 0.1);    
      controller->setParam("epsA", 0.2);    
      controller->setParam("rootE", 3);    
      controller->setParam("logaE", 0);    
      controller->setParam("sinerate", 15);  
      controller->setParam("phaseshift", 0.45);
    
      // AbstractWiring* wiring = new SelectiveOne2OneWiring(new ColorUniformNoise(0.2), new select_from_to(0,2));
      //OdeAgent* agent = new OdeAgent ( PlotOption(GuiLogger, Robot, 5) );
      //     agent->init ( controller , sphere1 , wiring );
      global.configs.push_back ( controller );

      AbstractWiring* wiring = new One2OneWiring ( new ColorUniformNoise(0.20) );
      OdeAgent* agent = new OdeAgent ( plotoptions );
      agent->init ( multisat , sphere1 , wiring );
      global.configs.push_back ( multisat );
      //      agent->setTrackOptions(TrackRobot(true, true, false, true, "ZSens", 50));
      global.agents.push_back ( agent );
    }
      
    showParams(global.configs);
  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
//     InvertMotorNStep* c = dynamic_cast<InvertMotorNStep*>(controller);
//     if(c){
//       matrix::Matrix m(3,1, dBodyGetLinearVel( sphere1->getMainPrimitive()->getBody())); 
//       c->setReinforcement(m.map(abs).elementSum()/3 - 1); 
//     }

  }
  
  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    char filename[256];
    if (down) { // only when key is pressed, not when released      
      switch ( (char) key )
	{
	case 'y' : dBodyAddForce ( sphere1->getMainPrimitive()->getBody() , 30 ,0 , 0 ); break;
	case 'Y' : dBodyAddForce ( sphere1->getMainPrimitive()->getBody() , -30 , 0 , 0 ); break;
	  //	case 'x' : dBodyAddTorque ( sphere1->getMainPrimitive()->getBody() , 0 , 10 , 0 ); break;
	  //	case 'X' : dBodyAddTorque ( sphere1->getMainPrimitive()->getBody() , 0 , -10 , 0 ); break;
	case 'x' : dBodyAddTorque ( sphere1->getMainPrimitive()->getBody() , 0 , 0, 30); break;
	case 'X' : dBodyAddTorque ( sphere1->getMainPrimitive()->getBody() , 0 , 0,-30); break;
	case 'n' : 
	  std::cout << "Please type a filename stem:";
	  std::cin >> filename;
	  if(multisat) multisat->storeSats(filename); 
	  break;
	default:
	  return false;
	  break;
	}
    }
    return false;
  }

//   virtual Node* makeGround(){ // is NOT used for shadowing!
//     float ir = 2000.0f;
//     float texscale =0.1;
//     Vec3Array *coords = new Vec3Array(4);
//     Vec2Array *tcoords = new Vec2Array(4);
//     Vec4Array *colors = new Vec4Array(1);

//     (*colors)[0].set(1.0f,1.0f,1.0f,1.0f);

//     (*coords)[0].set(-ir,-ir,0.0f);
//     (*coords)[1].set(-ir, ir,0.0f);
//     (*coords)[2].set( ir, ir,0.0f);
//     (*coords)[3].set( ir,-ir,0.0f);
//     (*tcoords)[0].set(-texscale*ir,-texscale*ir);
//     (*tcoords)[1].set(-texscale*ir, texscale*ir);
//     (*tcoords)[2].set( texscale*ir, texscale*ir);
//     (*tcoords)[3].set( texscale*ir,-texscale*ir);
    

//     Geometry *geom = new Geometry;

//     geom->setVertexArray( coords );

//     geom->setTexCoordArray( 0, tcoords );

//     geom->setColorArray( colors );
//     geom->setColorBinding( Geometry::BIND_OVERALL );

//     geom->addPrimitiveSet( new DrawArrays(PrimitiveSet::TRIANGLE_FAN,0,4) );

//     Texture2D *tex = new Texture2D;

//     tex->setImage(osgDB::readImageFile("Images/greenground_large.rgb"));
//     tex->setWrap( Texture2D::WRAP_S, Texture2D::REPEAT );
//     tex->setWrap( Texture2D::WRAP_T, Texture2D::REPEAT );

//     StateSet *dstate = new StateSet;
//     dstate->setMode( GL_LIGHTING, StateAttribute::OFF );
//     dstate->setTextureAttributeAndModes(0, tex, StateAttribute::ON );

//     dstate->setTextureAttribute(0, new TexEnv );

//     //     // clear the depth to the far plane.
//     //     osg::Depth* depth = new osg::Depth;
//     //     depth->setFunction(osg::Depth::ALWAYS);
//     //     depth->setRange(1.0,1.0);   
//     //     dstate->setAttributeAndModes(depth,StateAttribute::ON );

//     dstate->setRenderBinDetails(-1,"RenderBin");
//     geom->setStateSet( dstate );

//     Geode *geode = new Geode;
//     geode->addDrawable( geom );
//     geode->setName( "Ground" );

//     // add ODE Ground here (physical plane)
//     ground = dCreatePlane ( odeHandle.space , 0 , 0 , 1 , 0 );
//     dGeomSetCategoryBits(ground,Primitive::Stat);
//     dGeomSetCollideBits(ground,~Primitive::Stat);

//     return geode;
//   }

  
};

int main (int argc, char **argv)
{ 
  ThisSim sim;
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}
 
// with this seed the sphere does all sorts of things
//Use random number seed: 1181308527

// nice regular behaving sphere
// Use random number seed: 1181566764
 
