#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "simulation.h"

#include "arm2segm.h"


Arm2Segm::Arm2Segm(const OdeHandle& odeHandle):
  AbstractRobot::AbstractRobot(odeHandle){ 

  // prepare name;
  Configurable::insertCVSInfo(name, "$RCSfile$",
			      "$Revision$");

  created=false;


  factorMotors=1.0;
  factorSensors=20.0;
  avgMotor=0.3;
  maxMotorKraft=1;


  gelenkabstand =0.2;
  SOCKEL_LAENGE= 0.4;
  SOCKEL_BREITE= 0.1;
  SOCKEL_HOEHE =0.4;
  SOCKEL_MASSE =1;


  ARMDICKE=0.2;
  ARMLAENGE = 1.2;
  ARMABSTAND= 0.03;
  ARMMASSE = 0.001;





  /*
  initial_pos.x=0.0;
  initial_pos.y=0.0;
  initial_pos.z=0.0;
  */

  sensorno=armanzahl+1; 
  motorno=armanzahl+1;  
  segmentsno=armanzahl+1;

  for (int i=0; i<segmentsno; i++){
    old_angle[i]=0.0;
  }
 
  color.r=1;
  color.g=0;
  color.b=0;

};

/** sets actual motorcommands
    @param motors motors scaled to [-1,1] 
    @param motornumber length of the motor array
*/
void Arm2Segm::setMotors(const motor* motors, int motornumber){
  int len = (motornumber < motorno)? motornumber : motorno;
  double old_vel, new_vel;
  for (int i=0; i<len; i++){ 
    old_vel = dJointGetAMotorParam( jm[i] , dParamVel);
    new_vel = old_vel + avgMotor*(motors[i]-old_vel);
    dJointSetAMotorParam ( jm[i] , dParamVel , new_vel * factorMotors );
    //if (i==2)
    //dJointSetAMotorParam ( jm[i] , dParamVel , winkelgeschwindigkeit*MOTOR_WINKELGESCHWINDIGKEITSFAKTOR*0.75 );
  }
};


/** returns actual sensorvalues
    @param sensors sensors scaled to [-1,1] (more or less)
    @param sensornumber length of the sensor array
    @return number of actually written sensors
*/
int Arm2Segm::getSensors(sensor* sensors, int sensornumber){
  int len = (sensornumber < sensorno)? sensornumber : sensorno;
  double w;
  for ( int n = 0; n < len; n++ ){
    w=dJointGetAMotorAngle (jm[n],0)-old_angle[n];
    old_angle[n] = dJointGetAMotorAngle ( jm[n] , 0 );
    sensors[n]=w*factorSensors;      
    if ( ( w < -M_PI ) || ( w >  M_PI ) ){
      if ( w > M_PI ){
	sensors[n] = -(2*M_PI - w);
      }
      if ( w < -M_PI ){
	sensors[n] = (2*M_PI + w);
      }
    }
  }
  return len;
};

/** sets the vehicle to position pos, sets color to c, and creates robot if necessary
    @params pos desired position of the robot in struct Position
    @param c desired color for the robot in struct Color
*/
void Arm2Segm::place(Position pos, Color *c /*= 0*/){
  
 if (!c==0) {
    color=*c;
  }

  create(pos);
  /*
  if (!created){ 
    create(pos);
  }
  else{
    dBodySetPosition (object[1].body,pos.x ,pos.y +width*0.5,pos.z);
    dBodySetPosition (object[2].body,pos.x ,pos.y -width*0.5,pos.z);
    dBodySetPosition (object[0].body,pos.x ,pos.y           ,pos.z);    
  }
  */
};

/** returns position of robot 
    @return position robot position in struct Position  
*/
Position Arm2Segm::getPosition(){
  Position pos;
  const dReal* act_pos=dBodyGetPosition(object[0].body);
  pos.x=act_pos[0];
  pos.y=act_pos[1];
  pos.z=act_pos[2];
  return pos;
};

/** returns a vector with the positions of all segments of the robot
    @param poslist vector of positions (of all robot segments) 
    @return length of the list
*/
int Arm2Segm::getSegmentsPosition(vector<Position> &poslist){
  Position pos;
  for (int i=0; i<segmentsno; i++){
    const dReal* act_pos = dBodyGetPosition(object[i].body);
    pos.x=act_pos[0];
    pos.y=act_pos[1];
    pos.z=act_pos[2];
    poslist.push_back(pos);
  } 
  return segmentsno;
};  



/**
 * draws the vehicle
 */
void Arm2Segm::draw(){


  /**************************Zeichenabschnitt***********************/
  double box[3];
  //dsSetTexture (DS_WOOD);

  dsSetColor (color.r, color.g, color.b);
  
  box[0] = SOCKEL_LAENGE; box[1] = SOCKEL_BREITE; box[2] = SOCKEL_HOEHE;
  dsDrawBox ( dBodyGetPosition ( object[0].body ) , dBodyGetRotation ( object[0].body ) , box );

  box[0] = ARMLAENGE; box[1] = ARMDICKE; box[2] = ARMDICKE;
  dsDrawBox ( dBodyGetPosition ( object[1].body ) , dBodyGetRotation ( object[1].body ) , box );

  for ( int n = 2; n < armanzahl+1; n=n+1 ){
    dsDrawBox ( dBodyGetPosition ( object[n].body ) , dBodyGetRotation ( object[n].body ) , box );
  }

};

void Arm2Segm::mycallback(void *data, dGeomID o1, dGeomID o2){
  Arm2Segm* me = (Arm2Segm*)data;  
  if(isGeomInObjectList(me->object, me->segmentsno, o1) && isGeomInObjectList(me->object, me->segmentsno, o2)){
    return;
  }
}

void Arm2Segm::doInternalStuff(const GlobalData& globalData){}
bool Arm2Segm::collisionCallback(void *data, dGeomID o1, dGeomID o2){
   
 //checks if one of the collision objects is part of the robot
  if( o1 == (dGeomID)arm_space || o2 == (dGeomID)arm_space){
    dSpaceCollide(arm_space, this, mycallback);
    bool colwithme;  
    int i,n;  
    const int N = 10;
    dContact contact[N];
    n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
    for (i=0; i<n; i++){

      colwithme = false;  
      for (int j=0; j< segmentsno; j++){
	if( contact[i].geom.g1 == object[j].geom || contact[i].geom.g2 == object[j].geom){
	  colwithme = true;
	}
      }
//       if( contact[i].geom.g1 == object[0].geom || contact[i].geom.g2 == object[0].geom ||
// 	  contact[i].geom.g1 == object[1].geom || contact[i].geom.g2 == object[1].geom || 
// 	  contact[i].geom.g1 == object[2].geom || contact[i].geom.g2 == object[2].geom ){
// 	colwithme = true;
//       }
      if( colwithme){
	contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	  dContactSoftERP | dContactSoftCFM | dContactApprox1;
	contact[i].surface.slip1 = 0.005;
	contact[i].surface.slip2 = 0.005;
	contact[i].surface.mu = 0.5;
	contact[i].surface.soft_erp = 0.9;
	contact[i].surface.soft_cfm = 0.001;
	dJointID c = dJointCreateContact( world, contactgroup, &contact[i]);
	dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;	
      }
    }
    return true;
  }
  return false;
}


/** creates vehicle at desired position 
    @param pos struct Position with desired position
*/
void Arm2Segm::create(Position pos){
  if (created) {
    destroy();
  }
  // create arm space and add it to the top level space
  arm_space = dSimpleSpaceCreate (space);
  dSpaceSetCleanup (arm_space,0);


  //*************Koerperdefinitionsabschnitt**************
  //Sockel fuer die ROtation
  object[0].body = dBodyCreate ( world );
  dBodySetPosition ( object[0].body , 
		     pos.x + 0.5*SOCKEL_LAENGE , 
		     pos.y + 0.001 ,  
		     pos.z + SOCKEL_HOEHE* 0.5+ 0.01 );
  object[1].body = dBodyCreate ( world );
  dBodySetPosition ( object[1].body , 
		     pos.x + 0.5*ARMLAENGE + gelenkabstand , 
		     pos.y + ARMABSTAND + ARMDICKE , 
		     pos.z + SOCKEL_HOEHE*0.5 + 0.01 );

  //Arme
  for ( int n = 2; n < armanzahl+1; n=n+1 )
    {
      object[n].body = dBodyCreate ( world );
      dBodySetPosition ( object[n].body , 
			 dBodyGetPositionAll ( object[n-1].body , 1 ) + ARMLAENGE - gelenkabstand, 
			 dBodyGetPositionAll ( object[n-1].body , 2 ) + ARMDICKE + ARMABSTAND,  
			 dBodyGetPositionAll ( object[n-1].body , 3 ) );
      
    }

  dMass masse;
  //Aufbau der Massenverteilungsmatrix (hier fuer eine Box)
  dMassSetBox ( &masse , 1 , SOCKEL_LAENGE , SOCKEL_BREITE , SOCKEL_HOEHE );
  dMassAdjust ( &masse , SOCKEL_MASSE );
  dBodySetMass ( object[0].body , &masse );

  dMassSetBox ( &masse , 1 , ARMLAENGE , ARMDICKE , ARMDICKE );
  dMassAdjust ( &masse , ARMMASSE );
  dBodySetMass ( object[1].body , &masse );
  
  //Aenderung der Masse des Koerpers, aber beibehaltung der Massenverteilungsmatrix
  for ( int n = 2; n < armanzahl+1; n=n+1 ){
    dMassSetBox ( &masse , 1 , ARMLAENGE , ARMDICKE , ARMDICKE );
    dMassAdjust ( &masse , ARMMASSE );
    dBodySetMass ( object[n].body , &masse );
  }

  //**************Huellenfestlegungsabschnitt*************
  //Rotationssockel
  object[0].geom = dCreateBox ( arm_space , SOCKEL_LAENGE , SOCKEL_BREITE , SOCKEL_HOEHE );
  dGeomSetBody ( object[0].geom , object[0].body );
  //Arme
  object[1].geom = dCreateBox ( arm_space , ARMLAENGE , ARMDICKE , ARMDICKE );
  dGeomSetBody ( object[1].geom , object[1].body );
  
  for ( int n = 2; n < armanzahl+1; n=n+1 ){
    object[n].geom = dCreateBox ( arm_space , ARMLAENGE , ARMDICKE , ARMDICKE );
    dGeomSetBody ( object[n].geom , object[n].body );
  }


  //***************Motordefinitionsabschnitt**************
  
  jm[0] = dJointCreateAMotor ( world , 0 );
  dJointAttach ( jm[0] , object[0].body , 0 );
  dJointSetAMotorMode ( jm[0] , dAMotorEuler );
  //Dies sind die beiden festen Axen
  dJointSetAMotorAxis ( jm[0] , 0 , 1 , 0 , 0 , 1 );
  dJointSetAMotorAxis ( jm[0] , 2 , 2 , 1 , 0 , 0 );
  dJointSetAMotorParam ( jm[0] , dParamFMax , maxMotorKraft*7/*20*/ );

  jm[1] = dJointCreateAMotor ( world , 0 );
  dJointAttach ( jm[1] , object[1].body , object[0].body );
  dJointSetAMotorMode ( jm[1] , dAMotorEuler );
  //Dies sind die beiden festen Axen
  dJointSetAMotorAxis ( jm[1] , 0 , 1 , 0 , 1 , 0 );
  dJointSetAMotorAxis ( jm[1] , 2 , 2 , 0 , 0 , 1 );
  dJointSetAMotorParam ( jm[1] , dParamFMax , maxMotorKraft*2 /*10*/ );





  for ( int n = 2; n < armanzahl+1; n=n+1 ){
    jm[n] = dJointCreateAMotor ( world , 0 );
    dJointAttach ( jm[n] , object[n].body , object[n-1].body );
    dJointSetAMotorMode ( jm[n] , dAMotorEuler );
    //Dies sind die beiden festen Axen
    dJointSetAMotorAxis ( jm[n] , 0 , 1 , 0 , 1 , 0 );
    dJointSetAMotorAxis ( jm[n] , 2 , 2 , 0 , 0 , 1 );
    dJointSetAMotorParam ( jm[n] , dParamFMax , maxMotorKraft );
  }

  //*****************Join-Generierungsabschnitt***********
    //Sockel-Rotationsgelenk

  //  j[0] = dJointCreateHinge(world, 0);
  joint[0] = dJointCreateHinge (world,0);
  dJointAttach ( joint[0] , object[0].body , 0 );
  dJointSetHingeAnchor ( joint[0] , dBodyGetPositionAll ( object[0].body , 1 ) , 
			 dBodyGetPositionAll ( object[0].body , 2 ) , 
			 dBodyGetPositionAll ( object[0].body , 3 ) );
  dJointSetHingeAxis ( joint[0] , 0 , 0 , 1 );

  //Erstes Armgelenk (am Rotationssockel befestigt)
  joint[1] = dJointCreateHinge ( world , 0 );
  dJointAttach ( joint[1] , object[1].body , object[0].body );
  dJointSetHingeAnchor ( joint[1] , dBodyGetPositionAll ( object[0].body , 1 ) , 
			 dBodyGetPositionAll ( object[0].body , 2 ) , 
			 dBodyGetPositionAll ( object[1].body , 3 ) );
  dJointSetHingeAxis ( joint[1] , 0 , 1 , 0 );

  for ( int n = 2; n < armanzahl+1; n=n+1 ){
    joint[n] = dJointCreateHinge ( world , 0 );
    dJointAttach ( joint[n] , object[n].body , object[n-1].body );
    dJointSetHingeAnchor ( joint[n] , dBodyGetPositionAll ( object[n-1].body , 1 ) 
			   + (dBodyGetPositionAll ( object[n].body , 1 ) 
			      - dBodyGetPositionAll ( object[n-1].body , 1 ))/2 , 
			   dBodyGetPositionAll ( object[n].body , 2 ) ,  
			   dBodyGetPositionAll ( object[n].body , 3 ) );
    dJointSetHingeAxis ( joint[n] , 0 , 1 , 0 );
  }



  
  created=true;
}; 


/** destroys vehicle and space
 */
void Arm2Segm::destroy(){
  if (created){
    dSpaceDestroy(arm_space);
    for (int i=0; i<(armanzahl-1); i++){
      dBodyDestroy(object[i].body);
      dGeomDestroy(object[i].geom);
    }
  }
  created=false;
}


double Arm2Segm::dBodyGetPositionAll ( dBodyID basis , int para ){
    dReal Dpos[3];
    const dReal* pos = Dpos;

    pos = dBodyGetPosition ( basis );

    switch (para)
    {
        case 1: return pos[0]; break;
        case 2: return pos[1]; break;
        case 3: return pos[2]; break;	  
    }
    return 0;
    
}


int Arm2Segm::getParamList(paramkey*& keylist,paramval*& vallist) const{
  int number_params=4; // don't forget to adapt number params!
  keylist=(paramkey*)malloc(sizeof(paramkey)*number_params);
  vallist=(paramval*)malloc(sizeof(paramval)*number_params);
  keylist[0]="factorMotors";
  keylist[1]="factorSensors";  
  keylist[2]="avgMotor";  
  keylist[3]="maxMotorKraft";  

  vallist[0]=factorMotors;
  vallist[1]=factorSensors;
  vallist[2]=avgMotor;
  vallist[3]=maxMotorKraft;
  return number_params;
}


paramval Arm2Segm::getParam(paramkey key) const{
  if(!key) return 0.0;
  if(strcmp(key, "factorMotors")==0) return factorMotors; 
  else if(strcmp(key, "factorSensors")==0) return factorSensors; 
  else if(strcmp(key, "avgMotor")==0)    return avgMotor;  	
  else if(strcmp(key, "maxMotorKraft")==0)  return maxMotorKraft;  	
  else  return Configurable::getParam(key) ;
}

bool Arm2Segm::setParam(paramkey key, paramval val){
  if(!key) {
    fprintf(stderr, "%s: empty Key!\n", __FILE__);
    return false;
  }
  if(strcmp(key, "factorMotors")==0) factorMotors=val;
  else if(strcmp(key, "factorSensors")==0) factorSensors = val; 
  else if(strcmp(key, "avgMotor")==0)    avgMotor    = val;
  else if(strcmp(key, "maxMotorKraft")==0)  maxMotorKraft  = val;
  else return Configurable::setParam(key, val);
  return true;
}




