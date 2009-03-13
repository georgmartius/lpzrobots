/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius (at) informatik.uni-leipzig.de                                    *
 *    fhesse (at) informatik.uni-leipzig.de                                     *
 *    der (at) informatik.uni-leipzig.de                                        *
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
 *   Revision 1.15  2009-03-13 09:19:53  martius
 *   changed texture handling in osgprimitive
 *   new OsgBoxTex that supports custom texture repeats and so on
 *   Box uses osgBoxTex now. We also need osgSphereTex and so on.
 *   setTexture has to be called before init() of the primitive
 *
 *   Revision 1.14  2008/05/14 15:11:57  martius
 *   typo
 *
 *   Revision 1.13  2008/05/07 16:45:52  martius
 *   code cosmetics and documentation
 *
 *   Revision 1.12  2008/05/07 11:03:48  martius
 *   code cosmetics
 *
 *   Revision 1.11  2008/04/23 07:17:16  martius
 *   makefiles cleaned
 *   new also true realtime factor displayed,
 *    warning if out of sync
 *   drawinterval in full speed is 10 frames, independent of the speed
 *
 *   Revision 1.10  2007/11/07 13:21:16  martius
 *   doInternal stuff changed signature
 *
 *   Revision 1.9  2007/08/28 14:13:09  martius
 *   typo
 *
 *   Revision 1.8  2006/07/14 12:23:40  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.7.4.17  2006/06/29 16:39:55  robot3
 *   -you can now see bounding shapes if you type ./start -drawboundings
 *   -includes cleared up
 *   -abstractobstacle and abstractground have now .cpp-files
 *
 *   Revision 1.7.4.16  2006/06/25 17:00:32  martius
 *   Id
 *
 *   Revision 1.7.4.15  2006/06/25 16:57:14  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.7.4.14  2006/04/04 17:03:21  fhesse
 *   docu added
 *
 *   Revision 1.7.4.13  2006/04/04 14:13:24  fhesse
 *   documentation improved
 *
 *   Revision 1.7.4.12  2006/03/31 11:11:38  fhesse
 *   minor changes in docu
 *
 *   Revision 1.7.4.11  2006/02/01 18:33:40  martius
 *   use Axis type for Joint axis. very important, since otherwise Vec3 * pose is not the right direction vector anymore
 *
 *   Revision 1.7.4.10  2005/12/29 16:47:40  martius
 *   joint has getPosition
 *
 *   Revision 1.7.4.9  2005/12/15 17:04:08  martius
 *   Primitives are not longer inherited from OSGPrimitive, moreover
 *   they aggregate them.
 *   Joint have better getter and setter
 *
 *   Revision 1.7.4.8  2005/12/14 15:37:09  martius
 *   robots are working with osg
 *
 *   Revision 1.7.4.7  2005/12/13 18:11:39  martius
 *   still trying to port robots
 *
 *   Revision 1.7.4.6  2005/12/13 12:32:09  martius
 *   nonvisual joints
 *
 *   Revision 1.7.4.5  2005/12/12 23:41:30  martius
 *   added Joint wrapper
 *
 *   Revision 1.7.4.4  2005/12/11 23:35:08  martius
 *   *** empty log message ***
 *
 *   Revision 1.7.4.3  2005/12/06 10:13:25  martius
 *   openscenegraph integration started
 *
 *   Revision 1.7.4.2  2005/11/15 12:29:26  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.7.4.1  2005/11/14 17:37:17  martius
 *   moved to selforg
 *
 *   Revision 1.7  2005/11/09 13:24:42  martius
 *   added GPL
 *
 ***************************************************************************/
#include <assert.h>
#include <ode/ode.h>

// include primitives (box, spheres, cylinders ...)
#include "primitive.h"
#include "osgprimitive.h"

// include joints
#include "joint.h"

// include header file
#include "nimm4.h"

using namespace osg;


namespace lpzrobots {

  // constructor:
  // - give handle for ODE and OSG stuff
  // - size of robot, maximal used force and speed factor are adjustable
  // - sphereWheels switches between spheres or wheels as wheels
  //   (wheels are only drawn, collision handling is always with spheres)
  Nimm4::Nimm4(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
	       const std::string& name, 
	       double size/*=1.0*/, double force /*=3*/, double speed/*=15*/, 
	       bool sphereWheels /*=true*/)
    : // calling OdeRobots construtor with name of the actual robot
      OdeRobot(odeHandle, osgHandle, name, "$Id$")
  {   
    // robot is not created till now
    created=false;

    // choose color (here the color of the "Nimm Zwei" candy is used, 
    // where the name of the Nimm2 and Nimm4 robots comes from ;-)
    this->osgHandle.color = Color(2, 156/255.0, 0, 1.0f);
    
    // maximal used force is calculated from the force factor and size given to the constructor
    max_force   = force*size*size;
  
    // speed and type of wheels are set
    this->speed = speed;
    this->sphereWheels = sphereWheels;
  
    height=size;  
    length=size/2.5; // length of body
    width=size/2;  // diameter of body
    radius=size/6; // wheel radius
    wheelthickness=size/20; // thickness of the wheels (if wheels used, no spheres)
    cmass=8*size;  // mass of the body
    wmass=size;    // mass of the wheels
    sensorno=4;    // number of sensors
    motorno=4;     // number of motors
    segmentsno=5;  // number of segments of the robot
  };


  /** sets actual motorcommands
      @param motors motors scaled to [-1,1] 
      @param motornumber length of the motor array
  */
  void Nimm4::setMotors(const motor* motors, int motornumber){
    assert(created); // robot must exist
    // the number of controlled motors is minimum of
    // "number of motorcommands" (motornumber) and 
    // "number of motors inside the robot" (motorno)
    int len = (motornumber < motorno)? motornumber : motorno;

    // for each motor the motorcommand (between -1 and 1) multiplied with speed
    // is set and the maximal force to realize this command are set
    for (int i=0; i<len; i++){ 
      joint[i]->setParam(dParamVel2, motors[i]*speed);       
      joint[i]->setParam(dParamFMax2, max_force);
    }
  };

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int Nimm4::getSensors(sensor* sensors, int sensornumber){
    assert(created); // robot must exist

    // the number of sensors to read is the minimum of
    // "number of sensors requested" (sensornumber) and 
    // "number of sensors inside the robot" (sensorno)
    int len = (sensornumber < sensorno)? sensornumber : sensorno;

    // for each sensor the anglerate of the joint is red and scaled with 1/speed 
    for (int i=0; i<len; i++){
      sensors[i]=joint[i]->getPosition2Rate();
      sensors[i]/=speed;  //scaling
    }
    // the number of red sensors is returned 
    return len;
  };


  void Nimm4::place(const osg::Matrix& pose){
    // the position of the robot is the center of the body (without wheels)
    // to set the vehicle on the ground when the z component of the position is 0
    // width*0.6 is added (without this the wheels and half of the robot will be in the ground)    
    Matrix p2;
    p2 = pose * Matrix::translate(Vec3(0, 0, width*0.6)); 
    create(p2);    
  };


  /**
   * updates the osg notes
   */
  void Nimm4::update(){
    assert(created); // robot must exist
  
    for (int i=0; i<segmentsno; i++) { // update objects
      object[i]->update();
    }
    for (int i=0; i < 4; i++) { // update joints
      joint[i]->update();
    }

  };

  /** this function is called in each timestep. It should perform robot-internal checks, 
      like space-internal collision detection, sensor resets/update etc.
      @param global structure that contains global data from the simulation environment
  */
  void Nimm4::doInternalStuff(GlobalData& global){}

  /** creates vehicle at desired pose
      @param pose matrix with desired position and orientation
  */
  void Nimm4::create( const osg::Matrix& pose ){
    if (created) {  // if robot exists destroy it
      destroy();
    }
    // create car space
    odeHandle.createNewSimpleSpace(parentspace, true);
 
    OdeHandle wheelHandle(odeHandle);
    // make the material of the wheels a hard rubber
    wheelHandle.substance.toRubber(60);
    // create cylinder for main body
    // initialize it with ode-, osghandle and mass
    // rotate and place body (here by 90° around the y-axis)
    // use texture 'wood' for capsule 
    // put it into object[0]
    Capsule* cap = new Capsule(width/2, length);
    cap->getOSGPrimitive()->setTexture("Images/wood.rgb");
    cap->init(odeHandle, cmass, osgHandle);    
    cap->setPose(Matrix::rotate(M_PI/2, 0, 1, 0) * pose);    
    object[0]=cap;
    
    // create wheels
    for (int i=1; i<5; i++) {
      // create sphere with radius
      // and initialize it with odehandle, osghandle and mass
      // calculate position of wheels(must be at desired positions relative to the body)
      // rotate and place body (here by 90Deg around the x-axis)
      // set texture for wheels
      Sphere* sph = new Sphere(radius);
      sph->getOSGPrimitive()->setTexture("Images/wood.rgb");
      sph->init(wheelHandle, wmass, osgHandle.changeColor(Color(0.8,0.8,0.8)));    
      Vec3 wpos = Vec3( ((i-1)/2==0?-1:1)*length/2.0, 
			((i-1)%2==0?-1:1)*(width*0.5+wheelthickness), 
			-width*0.6+radius );
      sph->setPose(Matrix::rotate(M_PI/2, 0, 0, 1) * Matrix::translate(wpos) * pose);      
      object[i]=sph;
    }

    // generate 4 joints to connect the wheels to the body
    for (int i=0; i<4; i++) {
      Pos anchor(dBodyGetPosition (object[i+1]->getBody()));
      joint[i] = new Hinge2Joint(object[0], object[i+1], anchor, Axis(0,0,1)*pose, Axis(0,1,0)*pose);
      joint[i]->init(odeHandle, osgHandle, true, 2.01 * radius);
    } 
    for (int i=0; i<4; i++) {
      // set stops to make sure wheels always stay in alignment
      joint[i]->setParam(dParamLoStop, 0);
      joint[i]->setParam(dParamHiStop, 0);
    }

    created=true; // robot is created
  }; 


  /** destroys vehicle and space
   */
  void Nimm4::destroy(){
    if (created){
      for (int i=0; i<4; i++){
	if(joint[i]) delete joint[i]; // destroy bodies and geoms
      }
      for (int i=0; i<segmentsno; i++){
	if(object[i]) delete object[i]; // destroy bodies and geoms
      }
      odeHandle.deleteSpace(); // destroy space
    }
    created=false; // robot does not exist (anymore)
  }

}
