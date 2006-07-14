
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
 *   Revision 1.3  2006-07-14 14:36:43  martius
 *   cleanall target
 *   no unnecessary rebuild
 *
 *   Revision 1.2  2006/07/14 12:23:49  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.1  2006/07/13 13:18:20  der
 *   simulation with multiple snakes
 *
 *   Revision 1.2.4.3  2006/02/24 14:43:51  martius
 *   keys
 *
 *   Revision 1.2.4.2  2005/12/29 16:44:54  martius
 *   moved to osg
 *
 *   Revision 1.2.4.1  2005/11/15 12:29:59  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.2  2005/11/09 13:41:25  martius
 *   GPL'ised
 *
 ***************************************************************************/


#include <selforg/noisegenerator.h>

#include "simulation.h"
#include "odeagent.h"
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>
#include "playground.h"
#include "octaplayground.h"
#include "passivesphere.h"
#include "passivebox.h"

//#include <selforg/deprivation.h>
#include <selforg/invertnchannelcontroller.h>
//#include <selforg/invertmotorspace.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>
#include <selforg/dercontroller.h>

#include "schlangeservo.h"
#include "schlangeservo2.h"
#include "sphererobot3masses.h"

list<PlotOption> plotoptions;

using namespace lpzrobots;

class ThisSim : public Simulation {
public:
	

  /// start() is called at the start and should create all the object (obstacles, agents...).
  virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global){
    setCameraHomePos(Pos(-10.7854, -7.41751, 5.92078),  Pos(-50.6311, -5.00218, 0)); 

    /*     Playground* playground = new Playground(odeHandle, osgHandle, 
					    osg::Vec3(120, 0.2, 5.5),0.9);
    playground->setColor(Color(1,0.2,0,0.1));
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground);
    */
    /////////Neuer Playground klein innen 
    double diam=1.4; //internaldiameter=.9*diam, offset=1.0*internaldiameter; 

    OctaPlayground* playground2 = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(/*Diameter*/1.0*diam, 0.2,/*Height*/ 4), 12,false);
     playground2->setTexture("Images/whitemetal_farbig.rgb");
      playground2->setColor(Color(0.4,0.4,0.4,0.2));
      playground2->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
     global.obstacles.push_back(playground2);



  //   OctaPlayground* playground = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(/*Diameter=*/0.8*internaldiameter, 0.7, /*Height=*/1), 12,false);
//     playground->setColor(Color(1,0.2,0,0.9));
//     playground->setPosition(osg::Vec3(offset,0,0)); // playground positionieren und generieren
//      global.obstacles.push_back(playground);

// OctaPlayground* playground3 = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(/*Diameter=*/0.8*internaldiameter, 0.7, /*Height=*/.5), 12,false );
//     playground3->setColor(Color(1,0.2,0,0.9));
//     playground3->setPosition(osg::Vec3(-offset,0,0)); // playground positionieren und generieren
//      global.obstacles.push_back(playground3);
    //     controller->setParam("noiseB",0.0);

  

    // OctaPlayground* playground3 = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(/*Diameter*/4*diam, 5*diam,/*Height*/ 2), 12,false);
    // playground3->setColor(Color(.0,0.2,1.0,0.1));
    //playground3->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    //global.obstacles.push_back(playground2);


 OctaPlayground* playground4 = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(/*Diameter*/10.0*diam,.1,/*Height*/ 4), 12,true);
 // playground4->setTexture("Images/really_white.rgb");
 playground4->setColor(Color(.2,.2,.2,0.5));
 playground4->setGroundTexture("Images/really_white.rgb");
 playground4->setGroundColor(Color(255.0f/255.0f,200.0f/255.0f,21.0f/255.0f));
 playground4->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground4);

    /////////Neuer Playground Ende


 // Creation of passive boxes
  for(int i=0; i<-9; i++){
       PassiveBox* b = 
     new  PassiveBox(odeHandle, 
     		  osgHandle.changeColor(Color(184 / 255.0, 233 / 255.0, 237 / 255.0)), 
     		osg::Vec3(1.0, 1.0, 1.0), .2+i*0.1);
     b->setPosition(Pos(i*0.3-2, i*0.5-2,1.2*i+ 1.0)); 
     b->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(b);    
     }
 // Creation of passive spheres
 for(int i=0; i<-6; i++){
      PassiveSphere* s = 
	new PassiveSphere(odeHandle, 
			  osgHandle.changeColor(Color(184 / 255.0, 233 / 255.0, 237 / 255.0)),/*Diameter=*/.5+i*0.1,/*Mass=*/.5);
      s->setPosition(Pos(i*0.5-2, i*0.5-2,1.2*i+ 12)); 
      s->setTexture("Images/dusty.rgb");
      global.obstacles.push_back(s);    
    }

 // Creation of spherical robots: 
    for(int i=0; i<0; i++){
      OdeRobot* sphere1;
      Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();  
      conf.diameter=2;
      conf.axisZsensor=false;
      conf.axisXYZsensor=false;
      conf.irAxis1=true;
      conf.irAxis2=true;
      conf.irAxis3=true;
       sphere1 = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(0.0+.1*i,0.0,1.0)), 
      					 conf, "Sphere1", 0.4); 
       // sphere1 = new ForcedSphere(odeHandle, osgHandle, "FSphere");
    
       // sphere1->place ( Pos(-3,1/2,3+2*i));
      sphere1->place ( Pos(4,0,7+2*i));
      AbstractController* controller = new DerController();
      //AbstractController* controller = new InvertMotorNStep();
      controller->setParam("steps", 1);    
      controller->setParam("adaptrate", 0.0);  
      controller->setParam("nomupdate", 0.01);    
      controller->setParam("epsC", 0.01);    
      controller->setParam("epsA", 0.01);    
      controller->setParam("rootE", 3);     
      controller->setParam("factorB", 0.0);  
      
      DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
      c.useId = false;
      c.useFirstD = true;
      DerivativeWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise() );
      
      
       // One2OneWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );
      OdeAgent* agent = new OdeAgent ( plotoptions );
      agent->init ( controller , sphere1 , wiring );
      //  agent->setTrackOptions(TrackRobot(true, false, false, "ZSens_Ring10_11", 50));
      global.agents.push_back ( agent );
      global.configs.push_back ( controller );    
    }
 //creation of snakes 
      for(int i=0; i<1; i++){

      //****************/
      SchlangeConf conf = Schlange::getDefaultConf();
      conf.motorPower=.3;
      conf.segmNumber =14+2*i;//-i/2; 
      // conf.jointLimit=conf.jointLimit*3;
      conf.jointLimit=conf.jointLimit*2.0;
      conf.frictionGround=0.08+((double)i)/100;
      conf.frictionJoint=0.1;
      SchlangeServo2* schlange1; 
      if (i==0) {
	schlange1 = 
	//new SchlangeServo2 ( odeHandle, osgHandle.changeColor(Color(0.8, 0.3, 0.5)),
			     new SchlangeServo2 ( odeHandle, osgHandle.changeColor(Color(1.0, 1.0, 1.0)),
			      conf, "S1");
      } else {
	schlange1 = 
	//new SchlangeServo2 ( odeHandle, osgHandle.changeColor(Color(0.8, 0.3, 0.5)),
			     new SchlangeServo2 ( odeHandle, osgHandle.changeColor(Color(0.0, 0.0, 1.0)),
			      conf, "S1");
      }
      //Positionieren und rotieren 
      schlange1->place(osg::Matrix::rotate(M_PI/2, 0, 1, 0)*
		       osg::Matrix::translate(0.7-0.7*i,0,/*(i+1)**/(.2+conf.segmNumber)/*+2*/));
      schlange1->setTexture("Images/whitemetal_farbig.rgb");
      if (i==0) {
	schlange1->setHeadColor(Color(1.0,0,0));
      } else {
	schlange1->setHeadColor(Color(0,1.0,0));
      }
 

      //AbstractController *controller = new InvertNChannelController(100/*,true*/);  
      //  AbstractController *controller = new InvertMotorSpace(100/*,true*/);  
      //      AbstractController *controller = new InvertMotorNStep(); 
      AbstractController *controller = new DerController(); 
      // AbstractController *controller = new invertmotornstep();  
      //AbstractController *controller = new SineController();  
      //    AbstractController *controller = new InvertMotorNStep();  

      //     AbstractController *controller = new SineController();  
  
      //  AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.05)); //Only this line for one2Onewiring
      DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
      c.useId = true;
      c.useFirstD = true;
      DerivativeWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise() );
      


      //     AbstractWiring* wiring = new DerivativeWiring(c, new ColorUniformNoise(0.1));    
      //DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
      //c.useId = true;
      //c.useFirstD = true;
      //   c.derivativeScale=10;
      //   DerivativeWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise() );
      
     

      OdeAgent* agent = new OdeAgent(plotoptions);
      agent->init(controller, schlange1, wiring);
      global.agents.push_back(agent);
      global.configs.push_back(controller);
      global.configs.push_back(schlange1);
  
 
      global.odeConfig.setParam("controlinterval",2);
      global.odeConfig.setParam("gravity", -2); 

      controller->setParam("steps",1);
      controller->setParam("epsC",0.01);
      controller->setParam("epsA",0.01);
      controller->setParam("adaptrate",0.0);//0.005);
          controller->setParam("rootE",3);
          controller->setParam("logaE",0);

      // controller->setParam("desens",0.0);
         controller->setParam("s4delay",1.0);
         controller->setParam("s4avg",1.0);
    
         controller->setParam("factorB",0.0); 
	 controller->setParam("noiseB",0.0);

         controller->setParam("frictionjoint",0.01);
    
      }//creation of snakes End

    showParams(global.configs);
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


};

int main (int argc, char **argv)
{  
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;
}


 
  
