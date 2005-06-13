#ifndef __VEHICLE_H
#define __VEHICLE_H


class Vehicle : public AbstractRobot{
 public:  
  // some constants

  double length;  // chassis length
  double width;  // chassis width
  double height;   // chassis height
  double radius;  // wheel radius
  double cmass;    // chassis mass
  double wmass;    // wheel mass
  int sensorno;      //number of sensors
  int motorno;       // number of motors
  int segmentsno;    // number of motorsvehicle segments
  

  double pos_x, pos_y, pos_z;  // position of vehicle
  double max_force;            // maximal force for motors

  Object object[3];  // 1 cylinder, 2 wheels
  dJointID joint[2]; // joints between cylinder and each wheel

  dSpaceID car_space;

 public:
  
  Vehicle(dWorldID *w, dSpaceID *s):
    AbstractRobot::AbstractRobot(w,s){

    pos_x=0.0;
    pos_y=0.0;
    pos_z=0.5;
  
    color.r=0;
    color.g=1;
    color.b=1;
    
    max_force=0.05;

    length=0.07; 
    width=0.125; 
    height=0.2;  
    radius=0.064;
    cmass=1.0;  
    wmass=0.2;  
    sensorno=2; 
    motorno=2;  
    segmentsno=3;

  };

  virtual ~Vehicle(){
  };

  /** sets actual motorcommands
      @param motors motors scaled to [-1,1] 
      @param motornumber length of the motor array
  */
  virtual void setMotors(motor* motors, int motornumber){
    double tmp;
    int len = (motornumber < motorno)? motornumber : motorno;
    for (int i=0; i<len; i++){ 
      motors[i]*=20.0;  // scaling
      tmp=dJointGetHinge2Param(joint[i],dParamVel2);
      dJointSetHinge2Param(joint[i],dParamVel2,tmp + 0.5*(motors[i]-tmp) );
      dJointSetHinge2Param (joint[i],dParamFMax2,max_force);
    }
  };

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  virtual int getSensors(sensor* sensors, int sensornumber){
    int len = (sensornumber < sensorno)? sensornumber : sensorno;
    for (int i=0; i<len; i++){
      sensors[i]=dJointGetHinge2Angle2Rate(joint[i]);
      sensors[i]*=0.05;  //scaling
    }
    return len;
  };

  /** returns number of sensors
  */
  virtual int getSensorNumber(){
    return sensorno;
  };

  /** returns number of motors
  */
  virtual int getMotorNumber(){
    return motorno;
  };

  /** sets the position of robot to pos
      @param pos vector of desired position (x,y,z)
   */


  /** sets the vehicle to position  pos
      @param pos vector of desired position (x,y,z)
   */
  virtual void setPosition(const dVector3& pos){
  //virtual void setPosition(double x, double y, double z_){
    double z=pos[2]+radius; // to put wheels on ground, not in ground
    dBodySetPosition (object[1].body,pos[0] ,pos[1] +width*0.5,z);
    dBodySetPosition (object[2].body,pos[0] ,pos[1] -width*0.5,z);
    dBodySetPosition (object[0].body,pos[0],pos[1],z);    
    
  };

  /** returns position of vehicle 
      @param pos vector of position (x,y,z)
   */
  virtual void getPosition(dVector3& pos){
    const dReal* act_pos=dBodyGetPosition(object[0].body);
    for (int i=0; i<3; i++){
      pos[i]=act_pos[i];
    }
   
    pos[2]-=radius; // substract wheel radius, because vehicle stands on the ground
  };


  /** sets vehicle back to initial position   
   */
  virtual void setToInitialPosition(){
    dBodySetPosition (object[0].body,pos_x, pos_y,           pos_z+radius);
    dBodySetPosition (object[1].body,pos_x ,pos_y+ width*0.5,pos_z+radius );
    dBodySetPosition (object[2].body,pos_x ,pos_y -width*0.5,pos_z+radius );
  };
  
  /** returns a list with the positionvectors of all segments of the robot
      @param poslist list with positionvectors (of all robot segments) (free after use!)
      @return length of the list
  */
  virtual int getSegmentsPosition(dVector3*& poslist){
    poslist = (dVector3*)malloc(segmentsno * sizeof(dVector3));
    for (int i=0; i<segmentsno; i++){
      const dReal* act_pos = dBodyGetPosition(object[i].body);
      memcpy(poslist[i], act_pos,  4*sizeof(dReal));
    }
    return segmentsno;
  };
  

  //
  // welche von den folgenden in abstractrobot aufnehmen?
  //


  /** returns number of segments of the vehicle
  */
  virtual int getNumberSegments(){
    return segmentsno;
  };


  /**
   * draws the vehicle
   */
  virtual void draw(){
    dsSetColor (color.r,color.g,color.b); // set color for cylinder
    dsSetTexture (DS_WOOD);
    dsDrawCappedCylinder(dBodyGetPosition(object[0].body),dBodyGetRotation(object[0].body),length, width/2 );
    dsSetColor (1,1,1); // set color for wheels
    // draw wheels
    for (int i=1; i<3; i++) { 
      dsDrawCylinder (dBodyGetPosition(object[i].body), dBodyGetRotation(object[i].body),0.02f,radius);
    }
  };

  void create(){
    dMass m;
    // cylinder
    object[0].body = dBodyCreate (*world);
    dBodySetPosition (object[0].body,pos_x,pos_y,pos_z+radius);
    dQuaternion q;
    dQFromAxisAndAngle (q,0,1,0,M_PI*0.5);
    dBodySetQuaternion (object[0].body,q);
    
    dMassSetCappedCylinder(&m,1,1,width/2,length);
    dMassAdjust (&m,cmass);
    dBodySetMass (object[0].body,&m);
    object[0].geom = dCreateCCylinder (0,width/2,length);
    dGeomSetBody (object[0].geom, object[0].body);

    // wheel bodies
    for (int i=1; i<3; i++) {
      object[i].body = dBodyCreate (*world);
      dQuaternion q;
      dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
      dBodySetQuaternion (object[i].body,q);
      dMassSetSphere (&m,1,radius);
      dMassAdjust (&m,wmass);
      dBodySetMass (object[i].body,&m);
      object[i].geom = dCreateSphere (0,radius);
      dGeomSetBody (object[i].geom,object[i].body);
    }
    dBodySetPosition (object[1].body,pos_x ,pos_y+ width*0.5,pos_z+radius );
    dBodySetPosition (object[2].body,pos_x ,pos_y -width*0.5,pos_z+radius );


    for (int i=0; i<2; i++) {
      joint[i] = dJointCreateHinge2 (*world,0);
      dJointAttach (joint[i],object[0].body,object[i+1].body);
      const dReal *a = dBodyGetPosition (object[i+1].body);
      dJointSetHinge2Anchor(joint[i],a[0],a[1],a[2]);
      dJointSetHinge2Axis1 (joint[i],0,0,1);
      dJointSetHinge2Axis2 (joint[i],0,1,0);
    }
    for (int i=0; i<2; i++) {
      // set stops to make sure wheels always stay in alignment
      dJointSetHinge2Param (joint[i],dParamLoStop,0);
      dJointSetHinge2Param (joint[i],dParamHiStop,0);
    }
    // create car space and add it to the top level space
    car_space = dSimpleSpaceCreate (*space);
    dSpaceSetCleanup (car_space,0);
    for (int i=0; i<3; i++){
      dSpaceAdd (car_space,object[i].geom);
    }
  }; 




  void setColor(double r, double g, double b){
    color.r=r;
    color.g=g;
    color.b=b;
  };
  void setMaxForce(double m){
    max_force=m;
  };
  double getMaxForce(){
    return max_force;
  };

};


#endif
