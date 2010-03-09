/***************************************************************************
 *                                                                         *
 *  TruckMesh robot, an example for using Meshes for robot bodies          *
 *  basic code taken from the nimm4 robot                                  *
 *                                                                         *
 **************************************************************************/
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
 *   $Log$
 *   Revision 1.10  2010-03-09 11:53:41  martius
 *   renamed globally ode to ode-dbl
 *
 *   Revision 1.9  2009/03/13 09:19:53  martius
 *   changed texture handling in osgprimitive
 *   new OsgBoxTex that supports custom texture repeats and so on
 *   Box uses osgBoxTex now. We also need osgSphereTex and so on.
 *   setTexture has to be called before init() of the primitive
 *
 *   Revision 1.8  2008/05/07 16:45:52  martius
 *   code cosmetics and documentation
 *
 *   Revision 1.7  2008/01/17 09:57:40  der
 *   correct mesh filepath
 *
 *   Revision 1.6  2007/11/07 13:21:16  martius
 *   doInternal stuff changed signature
 *
 *   Revision 1.5  2007/09/06 18:48:00  martius
 *   createNewSimpleSpace used
 *
 *   Revision 1.4  2007/07/31 08:19:17  martius
 *   mesh without global
 *
 *   Revision 1.3  2006/09/21 22:09:58  martius
 *   collision for mesh
 *
 *   Revision 1.2  2006/07/14 12:23:42  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.3  2006/07/06 12:33:09  robot3
 *   -mass of the truck can be set by the constructor now
 *   -code cleaned up
 *   -all 6 wheels have now contact to the ground (instead of only 4)
 *
 *   Revision 1.1.2.2  2006/06/29 16:36:46  robot3
 *   -controller now gets 6 wheels for control
 *   -wheels are on the right position now
 *
 *   Revision 1.1.2.1  2006/06/27 15:19:52  robot3
 *   truckmesh uses a mesh for the body of the robot
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
#include <ode-dbl/ode.h>

// include primitives (box, spheres, cylinders, meshes ...)
#include "primitive.h"

// include joints
#include "joint.h"

// include header file
#include "truckmesh.h"
#include "osgprimitive.h" // get access to graphical (OSG) primitives

using namespace osg;


namespace lpzrobots {

  // constructor:
  // - give handle for ODE and OSG stuff
  // - size of robot, maximal used force and speed factor are adjustable
  TruckMesh::TruckMesh(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
		       const std::string& name,
		       double size/*=1.0*/, double force /*=3*/, double speed/*=15*/, double mass/*=1*/)
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

  
    height=size;  

    length=size*1.5; // length of the truck
    middlewidth=size/10; // for y axis, it's the middle of the truck
    middlelength=-size*0.326;
    width=size*0.4;  // width of the truck
    radius=size*0.0995; // wheel radius
    wheelthickness=size/20; // thickness of the wheels (if wheels used, no spheres)
    cmass=mass*size*8;  // mass of the body
    wmass=mass*size;    // mass of the wheels
    sensorno=6;    // number of sensors
    motorno=6;     // number of motors
    segmentsno=7;  // number of segments of the robot
  };


  /** sets actual motorcommands
      @param motors motors scaled to [-1,1] 
      @param motornumber length of the motor array
  */
  void TruckMesh::setMotors(const motor* motors, int motornumber){
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

    // another possibility is to set half of the difference between last set speed
    // and the actual desired speed as new speed; max_force is also set
    /*
      double tmp;
      int len = (motornumber < motorno)? motornumber : motorno;
      for (int i=0; i<len; i++){ 
      tmp=dJointGetHinge2Param(joint[i],dParamVel2);
      dJointSetHinge2Param(joint[i],dParamVel2,tmp + 0.5*(motors[i]*speed-tmp) );       
      dJointSetHinge2Param (joint[i],dParamFMax2,max_force);
      }
    */
  };

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int TruckMesh::getSensors(sensor* sensors, int sensornumber){
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


  void TruckMesh::place(const osg::Matrix& pose){
    // the position of the robot is the center of the body (without wheels)
    // to set the vehicle on the ground when the z component of the position is 0
    // width*0.6 is added (without this the wheels and half of the robot will be in the ground)    
    Matrix p2;
    p2 = pose * Matrix::translate(Vec3(0, 0, height*0.26)); 
    create(p2);    
  };


  /**
   * updates the osg notes
   */
  void TruckMesh::update(){
    assert(created); // robot must exist
  
    for (int i=0; i<segmentsno; i++) { // update objects
      object[i]->update();
    }
    for (int i=0; i < 6; i++) { // update joints
      joint[i]->update();
    }

  };

  /** things for collision handling inside the space of the robot can be done here
   */
  void TruckMesh::mycallback(void *data, dGeomID o1, dGeomID o2){
    // do collisions handling for collisions between parts inside the space of the robot here
    // this has no meaning for this robot, because collsions between wheels and body are ignored
    // but if parts of the robot can move against each other this is important

    // the follwing (not active) code part can be used to check if objects which had collisions 
    // are inside the list of objects of the robot
    /*  Nimm4* me = (Nimm4*)data;  
	if(isGeomInObjectList(me->object, me->segmentsno, o1) 
	&& isGeomInObjectList(me->object, me->segmentsno, o2)){
	return;
	}
    */
  }

  /** this function is called in each timestep. It should perform robot-internal checks, 
      like space-internal collision detection, sensor resets/update etc.
      @param global structure that contains global data from the simulation environment
  */
  void TruckMesh::doInternalStuff(GlobalData& global){}

  /** checks for internal collisions and treats them. 
   *  In case of a treatment return true (collision will be ignored by other objects 
   *  and the default routine)  else false (collision is passed to other objects and 
   *  (if not treated) to the default routine).
   */
  bool TruckMesh::collisionCallback(void *data, dGeomID o1, dGeomID o2){
    assert(created); // robot must exist
    return false;
  }


  /** creates vehicle at desired position 
      @param pos struct Position with desired position
  */
  void TruckMesh::create( const osg::Matrix& pose ){
    if (created) {  // if robot exists destroy it
      destroy();
    }
    // create car space and add it to the top level space
    odeHandle.createNewSimpleSpace(parentspace,true);
 
    // create mesh for main body
    // initialize it with ode-, osghandle and mass
    // rotate and place body (here by 90° around the y-axis)
    // use texture 'wood' for mesh
    // put it into object[0]
    Mesh* mesh = new Mesh("Meshes/dumptruck.osg",height/20.0f);
    mesh->getOSGPrimitive()->setTexture("Images/really_white.rgb");
    mesh->init(odeHandle, cmass, osgHandle);
    mesh->setPose(/*Matrix::rotate(M_PI/2, 0, 1, 0) */ pose);    
    object[0]=mesh;
    
    // create wheel bodies
    osgHandle.color= Color(1.0,1.0,1.0);
    for (int i=1; i<7; i++) {
      // create cylinder with radius and wheelthickness
      // and initializ it with odehandle, osghandle and mass
      // calculate position of wheels(must be at desired positions relative to the body)
      // rotate and place body (here by 90° around the x-axis)
      // set texture for wheels
      Cylinder* cyl=0;
      Vec3 wpos;
      if (i<3) { // back wheels
	cyl = new Cylinder(radius,wheelthickness*1.80);
	wpos = Vec3(middlelength-length*0.343,
		    middlewidth+((i-1)%2==0?-1.02:1)*width*0.35,
		    -height*0.302+radius );
      } 
      else if (i<5){ // middle wheels
	cyl = new Cylinder(radius,wheelthickness*1.80);
	wpos = Vec3(middlelength-length*0.201,
		    middlewidth+((i-1)%2==0?-1.02:1)*width*0.35,
		    -height*0.302+radius );
      }
      else if (i<7){ // front wheels
	cyl = new Cylinder(radius,wheelthickness*1.02);
	wpos = Vec3(middlelength+length*0.407,
		    middlewidth+((i-1)%2==0?-1.05:1)*width*0.387,
		    -height*0.302+radius); 
      }
      assert(cyl);
      cyl->getOSGPrimitive()->setTexture("Images/tire_full.rgb");
      cyl->init(odeHandle, wmass, osgHandle);    
      cyl->setPose(Matrix::rotate(M_PI/2, 1, 0, 0) * Matrix::translate(wpos) * pose);      
      object[i]=cyl;
    }

    // generate 6 joints to connect the wheels to the body
    for (int i=0; i<6; i++) {
      Pos anchor(dBodyGetPosition (object[i+1]->getBody()));
      joint[i] = new Hinge2Joint(object[0], object[i+1], anchor, Axis(0,0,1)*pose, Axis(0,1,0)*pose);
      joint[i]->init(odeHandle, osgHandle, true, 2.01 * wheelthickness);
    }
    for (int i=0; i<6; i++) {
      // set stops to make sure wheels always stay in alignment
      joint[i]->setParam(dParamLoStop, 0);
      joint[i]->setParam(dParamHiStop, 0);
    }

    created=true; // robot is created
  }; 


  /** destroys vehicle and space
   */
  void TruckMesh::destroy(){
    if (created){
      for (int i=0; i<4; i++){
	if(joint[i]) delete joint[i]; // destroy bodies and geoms
      }
      for (int i=0; i<segmentsno; i++){
	if(object[i]) delete object[i]; // destroy bodies and geoms
      }
      odeHandle.deleteSpace();
    }
    created=false; // robot does not exist (anymore)
  }

}
