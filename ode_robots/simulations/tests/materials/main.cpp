#/***************************************************************************
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
 *   $Log$
 *   Revision 1.4  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.3  2010/09/30 17:07:08  martius
 *   tests and vision experiments improved
 *
 *   Revision 1.2  2010/09/24 13:34:56  martius
 *   test for anisotrop friction
 *
 *   Revision 1.1  2010/09/22 07:48:55  martius
 *   test for materials moved here (from tests/)
 *
 *   Revision 1.11  2010/03/17 09:33:16  martius
 *   removed memory leaks and some small bugs
 *   valgrind suppression file is updated
 *
 *   Revision 1.10  2009/10/23 13:07:47  martius
 *   *** empty log message ***
 *
 *   Revision 1.9  2009/05/11 17:06:24  martius
 *   new schlange to test servos
 *
 *   Revision 1.8  2008/05/01 22:03:56  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.7  2007/07/03 13:06:18  martius
 *   *** empty log message ***
 *
 *   Revision 1.6  2007/06/08 15:37:22  martius
 *   random seed into OdeConfig -> logfiles
 *
 *   Revision 1.5  2007/04/03 16:35:54  der
 *   *** empty log message ***
 *
 *   Revision 1.4  2007/03/16 10:58:58  martius
 *   test of substances
 *
 *   Revision 1.3  2006/08/11 15:46:34  martius
 *   *** empty log message ***
 *
 *   Revision 1.2  2006/07/14 12:23:54  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.2  2006/06/25 17:01:57  martius
 *   remove old simulations
 *   robots get names
 *
 *   Revision 1.1.2.1  2006/06/16 21:35:50  martius
 *   test environment
 *
 *   Revision 1.14.4.15  2006/05/18 11:55:56  robot3
 *   made playground smaller (for shadowing issues)
 *
 *   Revision 1.14.4.14  2006/05/15 13:11:30  robot3
 *   -handling of starting guilogger moved to simulation.cpp
 *    (is in internal simulation routine now)
 *   -CTRL-F now toggles logging to the file (controller stuff) on/off
 *   -CTRL-G now restarts the GuiLogger
 *
 *   Revision 1.14.4.13  2006/04/25 09:06:16  robot3
 *   *** empty log message ***
 *
 *   Revision 1.14.4.12  2006/03/31 12:14:49  fhesse
 *   orange color for nimm robot
 *
 *   Revision 1.14.4.11  2006/03/31 11:27:53  fhesse
 *   documentation updated
 *   one sphere removed (to do fix problem with sphere placing)
 *
 *   Revision 1.14.4.10  2006/01/12 15:17:46  martius
 *   *** empty log message ***
 *
 *   Revision 1.14.4.9  2005/12/29 15:55:33  martius
 *   end is obsolete
 *
 *   Revision 1.14.4.8  2005/12/29 15:47:12  martius
 *   changed to real Sim class
 *
 *   Revision 1.14.4.7  2005/12/14 15:37:25  martius
 *   *** empty log message ***
 *
 *   Revision 1.14.4.6  2005/12/13 18:12:09  martius
 *   switched to nimm2
 *
 *   Revision 1.14.4.5  2005/12/11 23:35:08  martius
 *   *** empty log message ***
 *
 *   Revision 1.14.4.4  2005/12/09 16:53:17  martius
 *   camera is working now
 *
 *   Revision 1.14.4.3  2005/12/06 10:13:25  martius
 *   openscenegraph integration started
 *
 *   Revision 1.14.4.2  2005/11/24 16:19:12  fhesse
 *   include corrected
 *
 *   Revision 1.14.4.1  2005/11/15 12:30:07  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.14  2005/11/09 13:41:25  martius
 *   GPL'ised
 *
 ***************************************************************************/
#include <stdio.h>


// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>
#include <selforg/matrix.h>

// include simulation environment stuff
#include <ode_robots/simulation.h>

// used passive spheres and boxes
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/passivecapsule.h>


#include <ode_robots/joint.h>
#include <ode_robots/oneaxisservo.h>

// controller and robot
#include <selforg/sinecontroller.h>
#include <selforg/one2onewiring.h>
#include <ode_robots/odeagent.h>
#include <ode_robots/schlangeservo.h>

using namespace std;

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


class ThisSim : public Simulation {
public:

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(12.2217, 9.05786, 5.82483),  Pos(137.165, -12.3091, 0));
    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.setParam("noise", 0);
    global.odeConfig.setParam("simstepsize", 0.002);

    bool substances = false;
    bool transforms = false;
    bool motors     = false;
    bool schlange   = false;
    bool anisotrop_friction = true;

    s3=0;
    j=0;
    slider=0;

    if(transforms){
      if(1){
        s1 = new PassiveSphere(odeHandle, osgHandle, 0.5);
        s1->setPosition(osg::Vec3(10,2,2));
        global.obstacles.push_back(s1);

        s2 = new PassiveSphere(odeHandle, osgHandle, 0.5);
        s2->setPosition(osg::Vec3(10.8,2,2));
        global.obstacles.push_back(s2);

        Sphere* t = new Sphere(0.2);
        Transform* trans = new Transform(s2->getMainPrimitive(), t,osg::Matrix::translate(-0.5,0,0));
        trans->init(odeHandle, 0, osgHandle);
        OdeHandle o1 = odeHandle;
        o1.addIgnoredPair(trans->getGeom(), s1->getMainPrimitive()->getGeom());

        j = new HingeJoint(s1->getMainPrimitive(), s2->getMainPrimitive(), Pos(10.4,2,2.5), Axis(0,0,1));
        j->init(odeHandle, osgHandle);
      }

      if(1){
        s3  = new PassiveSphere(odeHandle, osgHandle, 0.5);
        s3->setPosition(osg::Vec3(10.8,5,0.6));
        global.obstacles.push_back(s3);
      }
    }
    if(substances){
      PassiveBox* b;
      OdeHandle handle2 = odeHandle;
      PassiveSphere* s;
      FixedJoint* fj;

      if(1){
        // Metal ground
        double y = 0;
        handle2.substance.toMetal(1);
        b = new PassiveBox(handle2, osgHandle.changeColor(Color(0.5,0.5,0.5)), osg::Vec3(7,1,1),100);
        b->setPosition(osg::Vec3(0,y,0));
        global.obstacles.push_back(b);
        fj= new FixedJoint(global.environment, b->getMainPrimitive());
        fj->init(odeHandle,osgHandle, false);

        handle2.substance.toMetal(1);
        s = new PassiveSphere(handle2, osgHandle.changeColor(Color(0.5,0.5,0.5)), 0.5);
        s->setPosition(osg::Vec3(-3,y,3));
        global.obstacles.push_back(s);

        handle2.substance.toPlastic(1);
        s = new PassiveSphere(handle2, osgHandle.changeColor(Color(1,1,1)), 0.5);
        s->setPosition(osg::Vec3(-1,y,3));
        global.obstacles.push_back(s);

        handle2.substance.toRubber(10);
        s = new PassiveSphere(handle2, osgHandle.changeColor(Color(0.1,0.1,0.1)), 0.5);
        s->setPosition(osg::Vec3(1,y,3));
        global.obstacles.push_back(s);

        handle2.substance.toFoam(5);
        s = new PassiveSphere(handle2, osgHandle.changeColor(Color(1,1,0)), 0.5);
        s->setPosition(osg::Vec3(3,y,3));
        global.obstacles.push_back(s);
      }
      if(1){
        double y = -3;
        // Plastik ground
        handle2.substance.toPlastic(0.8);
        b = new PassiveBox(handle2, osgHandle.changeColor(Color(1,1,1)), osg::Vec3(7,1,1),10);
        b->setPosition(osg::Vec3(0,y,0));
        global.obstacles.push_back(b);
        fj= new FixedJoint(global.environment, b->getMainPrimitive());
        fj->init(odeHandle,osgHandle, false);

        handle2.substance.toMetal(1);
        s = new PassiveSphere(handle2, osgHandle.changeColor(Color(0.5,0.5,0.5)), 0.5);
        s->setPosition(osg::Vec3(-3,y,3));
        global.obstacles.push_back(s);

        handle2.substance.toPlastic(1);
        s = new PassiveSphere(handle2, osgHandle.changeColor(Color(1,1,1)), 0.5);
        s->setPosition(osg::Vec3(-1,y,3));
        global.obstacles.push_back(s);

        handle2.substance.toRubber(10);
        s = new PassiveSphere(handle2, osgHandle.changeColor(Color(0.1,0.1,0.1)), 0.5);
        s->setPosition(osg::Vec3(1,y,3));
        global.obstacles.push_back(s);

        handle2.substance.toFoam(5);
        s = new PassiveSphere(handle2, osgHandle.changeColor(Color(1,1,0)), 0.5);
        s->setPosition(osg::Vec3(3,y,3));
        global.obstacles.push_back(s);
      }
      if(1){
        // Rubber ground
        double y = -6;
        handle2.substance.toRubber(10);
        b = new PassiveBox(handle2, osgHandle.changeColor(Color(0.1,0.1,0.1)), osg::Vec3(7,1,1),100);
        b->setPosition(osg::Vec3(0,y,0));
        global.obstacles.push_back(b);
        fj= new FixedJoint(global.environment, b->getMainPrimitive());
        fj->init(odeHandle,osgHandle, false);

        handle2.substance.toMetal(1);
        s = new PassiveSphere(handle2, osgHandle.changeColor(Color(0.5,0.5,0.5)), 0.5);
        s->setPosition(osg::Vec3(-3,y,3));
        global.obstacles.push_back(s);

        handle2.substance.toPlastic(1);
        s = new PassiveSphere(handle2, osgHandle.changeColor(Color(1,1,1)), 0.5);
        s->setPosition(osg::Vec3(-1,y,3));
        global.obstacles.push_back(s);

        handle2.substance.toRubber(10);
        s = new PassiveSphere(handle2, osgHandle.changeColor(Color(0.1,0.1,0.1)), 0.5);
        s->setPosition(osg::Vec3(1,y,3));
        global.obstacles.push_back(s);

        handle2.substance.toFoam(5);
        s = new PassiveSphere(handle2, osgHandle.changeColor(Color(1,1,0)), 0.5);
        s->setPosition(osg::Vec3(3,y,3));
        global.obstacles.push_back(s);
      }
      if(1){
        double y = -9;
        // Foam ground
        handle2.substance.toFoam(5);
        b = new PassiveBox(handle2, osgHandle.changeColor(Color(1,1,0)), osg::Vec3(7,1,1),10);
        b->setPosition(osg::Vec3(0,y,0));
        global.obstacles.push_back(b);
        fj= new FixedJoint(global.environment, b->getMainPrimitive());
        fj->init(odeHandle,osgHandle, false);

        handle2.substance.toMetal(1);
        s = new PassiveSphere(handle2, osgHandle.changeColor(Color(0.5,0.5,0.5)), 0.5);
        s->setPosition(osg::Vec3(-3,y,3));
        global.obstacles.push_back(s);

        handle2.substance.toPlastic(1);
        s = new PassiveSphere(handle2, osgHandle.changeColor(Color(1,1,1)), 0.5);
        s->setPosition(osg::Vec3(-1,y,3));
        global.obstacles.push_back(s);

        handle2.substance.toRubber(10);
        s = new PassiveSphere(handle2, osgHandle.changeColor(Color(0.1,0.1,0.1)), 0.5);
        s->setPosition(osg::Vec3(1,y,3));
        global.obstacles.push_back(s);

        handle2.substance.toFoam(5);
        s = new PassiveSphere(handle2, osgHandle.changeColor(Color(1,1,0)), 0.5);
        s->setPosition(osg::Vec3(3,y,3));
        global.obstacles.push_back(s);
      }
    }
    if(motors){
      int num=3;
      for(int k=0; k<=num; k++){
        PassiveCapsule* s = new PassiveCapsule(odeHandle, osgHandle.changeColor(Color(1,0,0)), 0.5, 2);
        s->setPosition(osg::Vec3(5,5,k*2+2));
        global.obstacles.push_back(s);
        caps.push_back(s);
      }
      j = new FixedJoint(global.environment, caps[num]->getMainPrimitive());
      j->init(odeHandle, osgHandle);
      for(int k=0; k<num; k++){
        Primitive* p1 = caps[k]->getMainPrimitive();
        Primitive* p2 = caps[k+1]->getMainPrimitive();
        OneAxisJoint* j1 = new HingeJoint(p1, p2,
                                          (p1->getPosition() + p2->getPosition()) /2,
                                          Axis(1,0,0));
        j1->init(odeHandle, osgHandle, false, 0);
        OneAxisServo* servo = new HingeServo(j1, -M_PI/2, M_PI/2, 10);
        servos.push_back(servo);
      }

    }
    if(schlange){
      SchlangeConf sc = SchlangeServo::getDefaultConf();
      sc.segmNumber=10;
      sc.useServoVel=true;
      sc.velocity = 20;
      sc.motorPower=20;
      OdeRobot* r  = new SchlangeServo(odeHandle, osgHandle, sc, "Schlange");
      r->place(osg::Matrix::rotate(M_PI/2,0,1,0)* osg::Matrix::translate(5,0,8));
      AbstractController* controller = new SineController(~0, SineController::SawTooth);
      OdeAgent* agent = new OdeAgent(global);
      agent->init ( controller , r , new One2OneWiring(new NoNoise()));
      global.agents.push_back(agent);
      global.configs.push_back(controller);
      global.configs.push_back(r);

      j = new FixedJoint(global.environment, r->getMainPrimitive());
      j->init(odeHandle, osgHandle);
    }

    if(anisotrop_friction){
      OdeHandle anisohandle = odeHandle;
      anisohandle.substance.toAnisotropFriction(.1, Axis(0,0,1));
      slider = new PassiveBox(anisohandle, osgHandle.changeColor(Color(0,1,1)),
                                    osg::Vec3(.2,.2,1),1);
      //      slider->setPose(osg::Matrix::rotate(M_PI/2,0,1,0)* osg::Matrix::translate(2,3,0.1));
      slider->setPose(osg::Matrix::translate(2,3,1));
      global.obstacles.push_back(slider);
    }



  }


  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    if(j) j->update();
    if(s3) {
      dBodyAddForce(s3->getMainPrimitive()->getBody(),1,0,0);
      matrix::Matrix v(1,3,dBodyGetLinearVel(s3->getMainPrimitive()->getBody()));
      cout << globalData.time << "\t " << v <<endl;
    }
    FOREACH(vector<OneAxisServo*>, servos, s){
      (*s)->set(sin(globalData.time));
    }
    if(slider){
      switch((globalData.sim_step/500)%5){
      case 0: slider->getMainPrimitive()->applyForce(4,0,0); break;
      case 1: slider->getMainPrimitive()->applyForce(-4,0,0); break;
      case 2: slider->getMainPrimitive()->applyForce(0,4,0);  break;
      case 3: slider->getMainPrimitive()->applyForce(0,-4,0); break;
      case 4: slider->getMainPrimitive()->applyTorque(0,0,3.4); break;
      }

    }

  };

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    bool hdld=false;
    if (down || 1) { // only when key is pressed, not when released
      switch ( (char) key )
        {
        case 'a': slider->getMainPrimitive()->applyForce(5,0,0); hdld = true;  break;
        case 'd': slider->getMainPrimitive()->applyForce(0,5,0); hdld = true;  break;
        case 'i':
          if(slider) slider->getMainPrimitive()->applyForce(200,0,0); hdld = true;  break;
        case 'k':
          if(slider) slider->getMainPrimitive()->applyForce(-200,0,0); hdld = true;  break;
        case 'j':
          if(slider) slider->getMainPrimitive()->applyForce(0,200,0); hdld = true; break;
        case ';':
          if(slider) slider->getMainPrimitive()->applyForce(0,-200,0); hdld = true; break;
        case 'n':
          if(slider) slider->getMainPrimitive()->applyTorque(0,0,100); hdld = true; break;
        case 'g': if(j) delete(j); j=0; hdld = true; break;
        default:
          break;
        }
    }
    return hdld;
  }


  PassiveSphere* s1;
  PassiveSphere* s2;
  PassiveSphere* s3;
  Joint* j;

  PassiveBox* slider;

  vector<PassiveCapsule*> caps;
  vector<OneAxisServo*> servos;
};


int main (int argc, char **argv)
{
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;
}

