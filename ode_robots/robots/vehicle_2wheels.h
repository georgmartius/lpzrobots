

class Vehicle{
 public:  
  // some constants

#define LENGTH 0.7/10.0	// chassis length
#define WIDTH 0.5/4.0	// chassis width
#define HEIGHT 0.2	// chassis height
#define RADIUS 0.18/2.8	// wheel radius
  //#define STARTZ 0.5	// starting height of chassis
#define CMASS 1.0		// chassis mass
#define WMASS 0.2	// wheel mass
  

  double start_x, start_y, start_z;
  double color_r, color_g, color_b;
  double max_force;
  


  // dynamics and collision objects (chassis, 3 wheels, environment)

  dBodyID body[4];
  dJointID joint[3];	// joint[0] is the front wheel

  dSpaceID car_space;
  dGeomID box[1];
  dGeomID sphere[3];



  // things that the user controls
  
  dReal speed,steer;	// user commands
 
  dSpaceID *space;
  dWorldID *world;


 public:
  
  Vehicle(dWorldID *w, dSpaceID *s){
    world=w;   	
    space=s;

    speed=0;
    steer=0;
  
    start_x=0.0;
    start_y=0.0;
    start_z=0.5;
  

  
    color_r=0;
    color_g=1;
    color_b=1;
    
    max_force=0.05;
    
  };

  ~Vehicle(){
  };

  void draw(){
    dsSetColor (color_r,color_g,color_b);
    dsSetTexture (DS_WOOD);
    //dReal sides[3] = {LENGTH,WIDTH,HEIGHT};
    //dsDrawBox (dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),sides);
    dsDrawCappedCylinder(dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),LENGTH, WIDTH/2 );
    dsSetColor (1,1,1);
    //for (int i=1; i<=3; i++) {
    for (int i=2; i<=3; i++) {
      dsDrawCylinder (dBodyGetPosition(body[i]), dBodyGetRotation(body[i]),0.02f,RADIUS);
    }
  };

  void create(){
    dMass m;


    // chassis body
    body[0] = dBodyCreate (*world);
    dBodySetPosition (body[0],start_x,start_y,start_z);
    //dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
      dQuaternion q;
      dQFromAxisAndAngle (q,0,1,0,M_PI*0.5);
      dBodySetQuaternion (body[0],q);

    dMassSetCappedCylinder(&m,1,1,WIDTH/2,LENGTH);
    dMassAdjust (&m,CMASS);
    dBodySetMass (body[0],&m);
    //box[0] = dCreateBox (0,LENGTH,WIDTH,HEIGHT);
    box[0] = dCreateCCylinder (0,WIDTH/2,LENGTH);
    dGeomSetBody (box[0],body[0]);

    // wheel bodies
    //for (int i=1; i<=3; i++) {
    for (int i=2; i<=3; i++) {
      body[i] = dBodyCreate (*world);
      dQuaternion q;
      dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
      dBodySetQuaternion (body[i],q);
      dMassSetSphere (&m,1,RADIUS);
      dMassAdjust (&m,WMASS);
      dBodySetMass (body[i],&m);
      sphere[i-1] = dCreateSphere (0,RADIUS);
      dGeomSetBody (sphere[i-1],body[i]);
    }
    //dBodySetPosition (body[1],0.5*LENGTH,0,STARTZ-HEIGHT*0.5);
    dBodySetPosition (body[2],start_x /*-0.5*LENGTH*/,start_y+ WIDTH*0.5,start_z /*-HEIGHT*0.5*/);
    dBodySetPosition (body[3],start_x /*-0.5*LENGTH*/,start_y -WIDTH*0.5,start_z /*-HEIGHT*0.5*/);

    // front wheel hinge
    /*
      joint[0] = dJointCreateHinge2 (world,0);
      dJointAttach (joint[0],body[0],body[1]);
      const dReal *a = dBodyGetPosition (body[1]);
      dJointSetHinge2Anchor (joint[0],a[0],a[1],a[2]);
      dJointSetHinge2Axis1 (joint[0],0,0,1);
      dJointSetHinge2Axis2 (joint[0],0,1,0);
    */

    // front and back wheel hinges
    //for (int i=0; i<3; i++) {
    for (int i=1; i<3; i++) {
      joint[i] = dJointCreateHinge2 (*world,0);
      dJointAttach (joint[i],body[0],body[i+1]);
      const dReal *a = dBodyGetPosition (body[i+1]);
      dJointSetHinge2Anchor (joint[i],a[0],a[1],a[2]);
      dJointSetHinge2Axis1 (joint[i],0,0,1);
      dJointSetHinge2Axis2 (joint[i],0,1,0);
    }

    // set joint suspension
    //for (int i=0; i<3; i++) {
    /*    for (int i=1; i<3; i++) {
      dJointSetHinge2Param (joint[i],dParamSuspensionERP,0.4);
      dJointSetHinge2Param (joint[i],dParamSuspensionCFM,0.8);
    }
    */
    // lock back wheels along the steering axis
    for (int i=1; i<3; i++) {
      // set stops to make sure wheels always stay in alignment
      dJointSetHinge2Param (joint[i],dParamLoStop,0);
      dJointSetHinge2Param (joint[i],dParamHiStop,0);
      // the following alternative method is no good as the wheels may get out
      // of alignment:
      //   dJointSetHinge2Param (joint[i],dParamVel,0);
      //   dJointSetHinge2Param (joint[i],dParamFMax,dInfinity);
    }

    // create car space and add it to the top level space
    car_space = dSimpleSpaceCreate (*space);
    dSpaceSetCleanup (car_space,0);
    dSpaceAdd (car_space,box[0]);
    //dSpaceAdd (car_space,sphere[0]);
    dSpaceAdd (car_space,sphere[1]);
    dSpaceAdd (car_space,sphere[2]);


  };


  void setMotors(double *x){
/*  
    // motor
    dJointSetHinge2Param (joint[0],dParamVel2,-speed);
    dJointSetHinge2Param (joint[0],dParamFMax2,0.1);
  
    // steering
    dReal v = steer - dJointGetHinge2Angle1 (joint[0]);
    if (v > 0.1) v = 0.1;
    if (v < -0.1) v = -0.1;
    v *= 10.0;
    dJointSetHinge2Param (joint[0],dParamVel,v);
    dJointSetHinge2Param (joint[0],dParamFMax,0.2);
    dJointSetHinge2Param (joint[0],dParamLoStop,-0.75);
    dJointSetHinge2Param (joint[0],dParamHiStop,0.75);
    dJointSetHinge2Param (joint[0],dParamFudgeFactor,0.1);
*/

    // motor
    //dJointSetHinge2Param (joint[1],dParamVel2,x[0]);
    double tmp;
    tmp=dJointGetHinge2Param(joint[1],dParamVel2);
    dJointSetHinge2Param(joint[1],dParamVel2,tmp + 0.5*(x[0]-tmp) );
    dJointSetHinge2Param (joint[1],dParamFMax2,max_force);



    //dJointSetHinge2Param (joint[2],dParamVel2,x[1]);
    tmp=dJointGetHinge2Param(joint[2],dParamVel2);
    dJointSetHinge2Param(joint[2],dParamVel2,tmp + 0.5*(x[1]-tmp) );
    dJointSetHinge2Param (joint[2],dParamFMax2,max_force);
  
  };

  void getSensors(double *x){
    x[0]=dJointGetHinge2Param (joint[1],dParamVel2);  //  falsch, gibt einfach nur wieder den Hingeparameter zurück
    x[1]=dJointGetHinge2Param (joint[2],dParamVel2);


  };

  void setInitialPosition(double x, double y, double z){
    start_x=x;
    start_y=y;
    start_z=z+0.3; // sonst Räder im Boden
  };

  void setPosition(double x, double y, double z_){
    double z=z_+RADIUS; // sonst Räder im Boden
    
    dBodySetPosition (body[2],x /*-0.5*LENGTH*/,y+ WIDTH*0.5,z /*-HEIGHT*0.5*/);
    dBodySetPosition (body[3],x /*-0.5*LENGTH*/,y -WIDTH*0.5,z /*-HEIGHT*0.5*/);
    dBodySetPosition (body[0],x,y,z);    
    
  };
  
  
  void setColor(double r, double g, double b){
    color_r=r;
    color_g=g;
    color_b=b;
  };
  void setMaxForce(double m){
    max_force=m;
  };
  double getMaxForce(){
    return max_force;
  };

};
