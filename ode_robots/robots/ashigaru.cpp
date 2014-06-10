/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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

#include <iostream>
#include <ode-dbl/ode.h>
#include <assert.h>
#include <osg/Matrix>

#include "ashigaru.h"
#include "irsensor.h"
#include "osgprimitive.h"

using namespace osg;
using namespace std;
using namespace ASHIGARU;

//#define DEBUG_MODE

namespace lpzrobots {

// this is the part of the impTransform
  // constructor
  ImpTransform::ImpTransform(Primitive* parent, Primitive* child, const Pose& pose):Transform(parent, child, pose){}

  // destructor
  ImpTransform::~ImpTransform(){}

  // function]
  // thorough this function, we can get position or rotation or something about child
  Pose ImpTransform::getChildPose(){
          Pose cPose;
          // the vector is multiplied from left hand, it was mentioned in another point
          cPose = pose * this->getPose();
          return cPose;
  }

  // constructor:
  // - give handle for ODE and OSG stuff, and default configuration
  Ashigaru::Ashigaru(const OdeHandle& odehandle, const OsgHandle& osgHandle,
                                                                                 const AshigaruConf& conf, const std::string& name)
                      :OdeRobot(odehandle, osgHandle, name, "0.1"), conf(conf), trunk()
        {
          //
          contactPoints=0;
          // robot not created up to now
          created=false;



          sensorno=ASHIGARU_SENSOR_MAX;
          motorno= ASHIGARU_MOTOR_MAX;// ASHIGARU_MOTOR_MAX   modifi


        // name the sensors
          // Leg joint angle sensor
          nameSensor(T0_as, "T0 angle sensor");
          nameSensor(T1_as, "T1 angle sensor");
          nameSensor(T2_as, "T2 angle sensor");
          nameSensor(C0_as, "C0 angle sensor");
          nameSensor(C1_as, "C1 angle sensor");
          nameSensor(C2_as, "C2 angle sensor");
          nameSensor(F0_as, "F0 angle sensor");
          nameSensor(F1_as, "F1 angle sensor");
          nameSensor(F2_as, "F2 angle sensor");

          // Leg contact sensors
          nameSensor(L0_fs, "Leg0 force sensor");
          nameSensor(L1_fs, "Leg1 force sensor");
          nameSensor(L2_fs, "Leg2 force sensor");

          // Leg joint angle sensor
          nameSensor(T0_ts, "T0 torque sensor");
          nameSensor(T1_ts, "T1 torque sensor");
          nameSensor(T2_ts, "T2 torque sensor");
          nameSensor(C0_ts, "C0 torque sensor");
          nameSensor(C1_ts, "C1 torque sensor");
          nameSensor(C2_ts, "C2 torque sensor");
          nameSensor(F0_ts, "F0 torque sensor");
          nameSensor(F1_ts, "F1 torque sensor");
          nameSensor(F2_ts, "F2 torque sensor");

          // attitude sensors
          nameSensor(POSE_r, "roll Pose sensor");
          nameSensor(POSE_p, "pitch Pose sensor");
          nameSensor(POSE_y, "yaw Pose sensor");

          // angular Velocity vector
          nameSensor(W_x, "angular vel x sensor");
          nameSensor(W_y, "angular vel y sensor");
          nameSensor(W_z, "angular vel z sensor");

          // global position of the Robot Center
          nameSensor(GPOS_Rx, "Global Pos of robot x");
          nameSensor(GPOS_Ry, "Global Pos of robot y");
          nameSensor(GPOS_Rz, "Global Pos of robot z");

          // global speed of the Center
          nameSensor(GSPD_Rx, "Global Spd of robot x");
          nameSensor(GSPD_Ry, "Global Spd of robot y");
          nameSensor(GSPD_Rz, "Global Spd of robot z");

          // global position of the COG
          nameSensor(GPOS_COGx, "Global COG Pos of robot x");
          nameSensor(GPOS_COGy, "Global COG Pos of robot y");
          nameSensor(GPOS_COGz, "Global COG Pos of robot z");

          // global position of the LegToe
          nameSensor(GPOS_L0x, "Global Leg0 toe Pos x");
          nameSensor(GPOS_L0y, "Global Leg0 toe Pos y");
          nameSensor(GPOS_L0z, "Global Leg0 toe Pos z");
          nameSensor(GPOS_L1x, "Global Leg1 toe Pos x");
          nameSensor(GPOS_L1y, "Global Leg1 toe Pos y");
          nameSensor(GPOS_L1z, "Global Leg1 toe Pos z");
          nameSensor(GPOS_L2x, "Global Leg2 toe Pos x");
          nameSensor(GPOS_L2y, "Global Leg2 toe Pos y");
          nameSensor(GPOS_L2z, "Global Leg2 toe Pos z");


        // name the motors
          nameMotor(T0_m, "T0 motor");
          nameMotor(T1_m, "T1 motor");
          nameMotor(T2_m, "T2 motor");
          nameMotor(C0_m, "C0 motor");
          nameMotor(C1_m, "C1 motor");
          nameMotor(C2_m, "C2 motor");
          nameMotor(F0_m, "F0 motor");
          nameMotor(F1_m, "F1 motor");
          nameMotor(F2_m, "F2 motor");

  };


  Ashigaru::~Ashigaru(){
    destroy();
  }

  // name sensor
  void Ashigaru::nameSensor(const int sensorNo, const char* name) {
          addInspectableDescription("x[" + std::itos(sensorNo) + "]", name);
  }

  // name motor
  void Ashigaru::nameMotor(const int motorNo, const char* name) {
          addInspectableDescription("y[" + std::itos(motorNo) + "]", name);
  }


  // Configuration get function static
  AshigaruConf Ashigaru::getDefaultConf(){
          AshigaruConf conf;
                // configuration of rate
                conf.rate = 1.; // we multiple 1 with the length
                // configuration of mass rate
                conf.massRate = 1.;
                // configuration of connect length
                conf.connectLength = 0.0519 + 0.0644 + 0.068 + 0.0275;

                // configuration for dynamixel
                conf.dyna.width = 0.025 /* m */ * conf.rate;
                conf.dyna.length = 0.050 * conf.rate;
                conf.dyna.height = 0.036 * conf.rate;
                conf.dyna.length_axis_to_center = 0.0135 * conf.rate;
                conf.dyna.length_from_axis_to_tip = 0.0265 * conf.rate;
                conf.dyna.mass = 0.0535 * conf.massRate;// kg //0.001;//

                // config. of body
                conf.body.height = 0.003 * conf.rate; // m
                conf.body.length = 0.060 * conf.rate;
                conf.body.mass = 0.160* conf.massRate; // kg 0.001;//

                // config. of foot
                conf.foot.height = 0.002 * conf.rate; // m
                conf.foot.length = 0.120 * conf.rate;
                conf.foot.width  = 0.038 * conf.rate;
                conf.foot.footRadius = 0.007 * conf.rate;

                conf.foot.mass = 0.035* conf.massRate; // kg 0.001;//

                // config. of joint length
                conf.jLength.length_center_to_TCJ = 0.0644 * conf.rate; //m
                conf.jLength.length_TCJ_to_CTJ = 0.068 * conf.rate; //m
                conf.jLength.length_CTJ_to_FTJ = 0.068 * conf.rate;
                conf.jLength.length_FTJ_to_Toe = 0.125 * conf.rate;

                // config. of servo parameter
                conf.servoParam.TC_angle_MIN = -M_PI * 90. / 180.;
                conf.servoParam.TC_angle_MAX = M_PI * 90. / 180.;
                conf.servoParam.CT_angle_MIN = - 2. * M_PI * 90. / 180.;
                conf.servoParam.CT_angle_MAX = 0.; //M_PI * 90. / 180.;
                conf.servoParam.FT_angle_MIN = 0.;//-M_PI * 90. / 180.;
                conf.servoParam.FT_angle_MAX = 2. * M_PI * 90. / 180.;

                conf.servoParam.power = 20;//20.;
                conf.servoParam.damp = 0.;
                conf.servoParam.maxVel = 1.7 * 1.961 * M_PI;

                // special parameter
                //  to make the connected leg rigid
                conf.specialParam.conectedLegNum = 0;
                conf.specialParam.servoPower = 1000.;

                // all mass calculation
                conf.wholeMass = conf.body.mass + 9. * conf.dyna.mass + 3. * conf.foot.mass;

                return conf;
  }

  // The function to get the motor Name
  Ashigaru::MotorName Ashigaru::getMotorName(LegPos leg, LegJointType joint){
          if (leg == L0){
                  if(joint == TC){
                          return T0_m;
                  }else if(joint == CT){
                          return C0_m;
                  }else if(joint == FT){
                          return F0_m;
                  }
          }else if(leg == L1){
                  if(joint == TC){
                          return T1_m;
                  }else if(joint == CT){
                          return C1_m;
                  }else if(joint == FT){
                          return F1_m;
                  }
          }else if(leg == L2){
                  if(joint == TC){
                          return T2_m;
                  }else if(joint == CT){
                          return C2_m;
                  }else if(joint == FT){
                          return F2_m;
                  }
          }

          return ASHIGARU_MOTOR_MAX;

  }

  /** gets Primitives of Leg tibia
      @param LegPos Number
  */
  Primitive* Ashigaru::getTibiaPrimitive(LegPos leg){
          assert(created);
          if(leg < LEG_POS_MAX){
                  return legs[leg].tibia;
          }else{
                  return 0;
          }
  }


  /** sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  void Ashigaru::setMotorsIntern(const double* motors, int motornumber){
    assert(created);
    assert(motornumber == motorno);

    for (MotorMap::iterator it = servos.begin(); it != servos.end(); it++) {
      MotorName const name = it->first;
      OneAxisServo * const servo = it->second;
      //We set the motor command in the servo
      if(servo){
        servo->set(motors[name]);
      }
    }
  }

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
//   sensor ir_old[4];
//   sensor ir_tmp[4];

  int Ashigaru::getSensorsIntern(sensor* sensors, int sensornumber){
    assert(created);
    assert(sensornumber == getSensorNumberIntern());

     // angle sensors
     //
     sensors[T0_as] = servos[T0_m] ? servos[T0_m]->get() : 0;
     sensors[T1_as] = servos[T1_m] ? servos[T1_m]->get() : 0;
     sensors[T2_as] = servos[T2_m] ? servos[T2_m]->get() : 0;

     sensors[C0_as] = servos[C0_m] ? servos[C0_m]->get() : 0;
     sensors[C1_as] = servos[C1_m] ? servos[C1_m]->get() : 0;
     sensors[C2_as] = servos[C2_m] ? servos[C2_m]->get() : 0;

     sensors[F0_as] = servos[F0_m] ? servos[F0_m]->get() : 0;
     sensors[F1_as] = servos[F1_m] ? servos[F1_m]->get() : 0;
     sensors[F2_as] = servos[F2_m] ? servos[F2_m]->get() : 0;

     // Contact sensors
     sensors[L0_fs] = legContactSensors[L0] ? legContactSensors[L0]->get() : 0;
     sensors[L1_fs] = legContactSensors[L1] ? legContactSensors[L1]->get() : 0;
     sensors[L2_fs] = legContactSensors[L2] ? legContactSensors[L2]->get() : 0;

     // Torque sensors
     //motorTorqSensors[T0_m]->update();
     sensors[T0_ts] = getTorqueData(motorTorqSensors[T0_m]);
     sensors[T1_ts] = getTorqueData(motorTorqSensors[T1_m]);
     sensors[T2_ts] = getTorqueData(motorTorqSensors[T2_m]);
     sensors[C0_ts] = getTorqueData(motorTorqSensors[C0_m]);
     sensors[C1_ts] = getTorqueData(motorTorqSensors[C1_m]);
     sensors[C2_ts] = getTorqueData(motorTorqSensors[C2_m]);
     sensors[F0_ts] = getTorqueData(motorTorqSensors[F0_m]);
     sensors[F1_ts] = getTorqueData(motorTorqSensors[F1_m]);
     sensors[F2_ts] = getTorqueData(motorTorqSensors[F2_m]);

     //Pose sensor
     osg::Vec3d a = this->convert_Quat_to_RollPitchYaw(this->getMainPrimitive()->getPose().getRotate());
     sensors[POSE_r] = a[0]; // rad angle
     sensors[POSE_p] = a[1];
     sensors[POSE_y] = a[2];

     //angular velocity of center of the robot
     a = this->getMainPrimitive()->getAngularVel();
     sensors[W_x] = a[0]; // angular vel vector
     sensors[W_y] = a[1];
     sensors[W_z] = a[2];

     // global position of robot center
     a = this->getMainPrimitive()->getPosition();
     sensors[GPOS_Rx] = a[0]; // global position of the robot
     sensors[GPOS_Ry] = a[1];
     sensors[GPOS_Rz] = a[2];

     // global speed of robot center
     //a = this->legs[L0].shoulder->getPose().getTrans();
     a = this->getMainPrimitive()->getVel();
     sensors[GSPD_Rx] = a[0]; // global spd of the robot
     sensors[GSPD_Ry] = a[1];
     sensors[GSPD_Rz] = a[2];

     // global position of the COG
     a = this->calc_COGPosition();
         sensors[GPOS_COGx] = a[0]; // global cog of the robot
     sensors[GPOS_COGy] = a[1];
     sensors[GPOS_COGz] = a[2];

     // global position of the leg toe
     a = this->legs[L0].foot->getChildPose().getTrans();
     sensors[GPOS_L0x] = a[0];
     sensors[GPOS_L0y] = a[1];
     sensors[GPOS_L0z] = a[2];

     a = this->legs[L1].foot->getChildPose().getTrans();
     sensors[GPOS_L1x] = a[0];
     sensors[GPOS_L1y] = a[1];
     sensors[GPOS_L1z] = a[2];

     a = this->legs[L2].foot->getChildPose().getTrans();
     sensors[GPOS_L2x] = a[0];
     sensors[GPOS_L2y] = a[1];
     sensors[GPOS_L2z] = a[2];

    return ASHIGARU_SENSOR_MAX;
  };


  void Ashigaru::placeIntern(const osg::Matrix& pose){
    // the position of the robot is the center of the body (without wheels)
    // to set the vehicle on the ground when the z component of the position is 0
    // length of foot is added (without this the wheels and half of the robot will be in the ground)
    Matrix p2;
    p2 = pose * Matrix::translate(Vec3(0, 0, conf.foot.length + conf.foot.footRadius * 2. - conf.dyna.width /2.));
    create(p2);

  };

  /** returns a vector with the positions of all segments of the robot
      @param poslist vector of positions (of all robot segments)
      @return length of the list
  */
  int Ashigaru::getSegmentsPosition(std::vector<Position> &poslist){
    assert(created);
    for (int i=0; i<3; i++){
      poslist.push_back(Position(dBodyGetPosition(objects[i]->getBody())));
    }
    return 3;
  };

  /**
   * updates the osg notes and sensorbank
   */
  void Ashigaru::update() {
    OdeRobot::update();
    assert(created); // robot must exist

    // update sensorbank with infrared sensors
    //irSensorBank.update();
    this->legs[L0].shoulderBox->update();

    //update contact sensors
    for (int i = 0; i < LEG_POS_MAX; i++) {
       if (legContactSensors[LegPos(i)])
         legContactSensors[LegPos(i)]->update();
    }

  }

  void Ashigaru::sense(GlobalData& globalData){
    OdeRobot::sense(globalData);
    
      // reset contact sensors
    for (int i = 0; i < LEG_POS_MAX; i++) {
      if (legContactSensors[LegPos(i)])
        legContactSensors[LegPos(i)]->sense(globalData);
    }
    motorTorqSensors[T0_m]->sense(globalData);
    motorTorqSensors[T1_m]->sense(globalData);
    motorTorqSensors[T2_m]->sense(globalData);
    motorTorqSensors[C0_m]->sense(globalData);
    motorTorqSensors[C1_m]->sense(globalData);
    motorTorqSensors[C2_m]->sense(globalData);
    motorTorqSensors[F0_m]->sense(globalData);
    motorTorqSensors[F1_m]->sense(globalData);
    motorTorqSensors[F2_m]->sense(globalData);
  }


  //May be it is called every step??  -> Frank: yes
  void Ashigaru::doInternalStuff(GlobalData& globalData){
    // update statistics
    //position = getPosition();

    // reset ir sensors to maximum value
    //irSensorBank->reset();


  }

  /** creates vehicle at desired position
      @param pos struct Position with desired position
  */
  void Ashigaru::create(const osg::Matrix& pose){
    if (created) {
      destroy();
    }

#ifdef DEBUG_MODE
    cout << "hello, This is the debug mode" << endl;
    cout << "@@@@ First, We create the robot!! @@@@" << endl;
#endif

    /*********************************************************************************:
     *  The ordinal configuration
     *     Environment, color, feature etc/
    ***********************************************************************************/

    // we want legs colliding with other legs, so we set internal collision
    // flag to "false".
    odeHandle.createNewSimpleSpace(parentspace,false);

    // Ashigaru color ;-)
    osgHandle.color = Color(0/255., 30./255., 0./255., 1.0f);

    // Joint color ;-)
    OsgHandle dynaHandle(osgHandle);
    dynaHandle.color = Color(204/255., 51/255., 0./255., 1.0f);
    //dynaHandle.color = Color(0/255., 0/255., 0./255., 1.0f);

    // Tibia Color (colored by leg)
    OsgHandle tibiaHandle[3] = {osgHandle, osgHandle, osgHandle};
    tibiaHandle[0].color = Color(0./255., 30./255., 10./255., 1.0f);
    tibiaHandle[1].color = Color(0./255., 30./255., 90./255., 1.0f);
    tibiaHandle[2].color = Color(0./255., 30./255., 170./255., 1.0f);

    // change Material substance
    OdeHandle odeHandleBody(odeHandle);
    odeHandleBody.substance.toMetal(3.0);

    // get a representation of the origin
     // By multipling this parameter to pose, we can get only the position data of pose
    const Pos nullpos(0, 0, 0);


    /*********************************************************************************:
    *  Create Body Structure
    *     Trunk, Leg etc.
    ***********************************************************************************/

    /*********************************************
     *  Main Frame */
    // Make the boxes connect each other to make hexagon
    /**********************************************/
#ifdef DEBUG_MODE
    cout << "@@@@ Creation of Body frame starts!! @@@@" << endl;
#endif

      // make the rectangle boxes to make hexagon (they are connected each other and make hexagon)
      // make 6 rectangle plate objects
      for(int i=0; i< 6;i++){
              trunk.tPlate[i] = new Box( conf.body.length * sqrt(3.), conf.body.length, conf.body.height);
              trunk.tPlate[i]->getOSGPrimitive()->setTexture("Images/wood.rgb");
              // This function (setMass) seems not to be implemented
              //trunk.tPlate[i]->setMass(conf.body.mass / 6.);
      }

      // First object should be initialized (I choose the rectangle one)
      trunk.tPlate[0]->init(odeHandle, conf.body.mass / 6., osgHandle);
      trunk.tPlate[0]->setPose( pose );
       // add it to Object
      objects.push_back(trunk.tPlate[0]);

      // we transform these 3 plates each other to make the hexagon
       // Transformation to make hexagonal lower plate
      // Transformation 1
      trunk.tTrans[0] = new ImpTransform(trunk.tPlate[0], trunk.tPlate[1],
                      Matrix::rotate(M_PI/3.0, Vec3(0, 0, 1)) *
                      Matrix::translate(0, 0, 0));
      trunk.tTrans[0]->init(odeHandle, conf.body.mass / 6., osgHandle);
      // add it to Object
      objects.push_back(trunk.tTrans[0]);

      //transformation 2
      trunk.tTrans[1] = new ImpTransform(trunk.tPlate[0], trunk.tPlate[2],
                            Matrix::rotate(-M_PI/3.0, Vec3(0, 0, 1)) *
                            Matrix::translate(0, 0, 0));
      trunk.tTrans[1]->init(odeHandle, conf.body.mass / 6., osgHandle);
      // add it to Object
      objects.push_back(trunk.tTrans[1]);

      //2nd Body
      // to make second hexagon upper plate, we will connect the plates each other.
      // Transformation 3~5
      for(int i =0;i<3;i++){
              if(i == 0){
                      // just for memorlizing transForm
                      trunk.tUpTrans = new ImpTransform(trunk.tPlate[0], trunk.tPlate[i+3],
                                      Matrix::rotate(M_PI/3.0 * (double)i, Vec3(0, 0, 1)) *
                                      Matrix::translate(0, 0, conf.dyna.height));
                      trunk.tTrans[ i+2 ] = trunk.tUpTrans;
              }else{
                      trunk.tTrans[ i+2 ] = new ImpTransform(trunk.tPlate[0], trunk.tPlate[i+3],
                                  Matrix::rotate(M_PI/3.0 * (double)i, Vec3(0, 0, 1)) *
                                  Matrix::translate(0, 0, conf.dyna.height));
              }
              trunk.tTrans[ i+2 ]->init(odeHandle, conf.body.mass / 6., osgHandle);
              // add it to Object
              objects.push_back(trunk.tTrans[i+2]);
      }


    /*********************************************
       LEGs  **sholder, coxa femur tibia
       //
        **********************************************/
#ifdef DEBUG_MODE
    cout << "@@@@ Creation of Leg frame starts!! @@@@" << endl;
#endif

      // Useful Parameter to make robot model

      // Transition Matrix from origin of this robot to center of the robot
      //  notice: the orgin of the robot is on the center of the lower hexagonal plate
      osg::Matrix trans_rO_to_rC = Matrix::translate( 0, 0, conf.dyna.height / 2.);

      // Trans Matrix from center of the robot to Shoulder Dynamixel center
       // The horizontal length between robot center and Shoulder Dynamixel center
       double length_rC_to_sC = (conf.jLength.length_center_to_TCJ - conf.dyna.length_axis_to_center);
       // Matrix
       osg::Matrix trans_rC_to_sC =  Matrix::translate( length_rC_to_sC, 0, 0);

      // Trans Matrix from center of the robot to Coxa Dynamixel center
       // The horizontal length between robot center and Coxa Dynamixel center
       double length_rC_to_cC = length_rC_to_sC + conf.jLength.length_TCJ_to_CTJ;
       // Matrix
           osg::Matrix trans_rC_to_cC =  Matrix::translate( length_rC_to_cC, 0, 0);

          // Trans Matrix from center of the robot to Femur Dynamixel center
       // The horizontal length between robot center and Femur Dynamixel center
           double length_rC_to_fC_x = conf.jLength.length_center_to_TCJ + conf.jLength.length_TCJ_to_CTJ;
           double length_rC_to_fC_z = conf.jLength.length_CTJ_to_FTJ - conf.dyna.length_axis_to_center;
       //double length_rC_to_fC = length_rC_to_cC + conf.jLength.length_CTJ_to_FTJ;
       // Matrix
       osg::Matrix trans_rC_to_fC =  Matrix::translate( length_rC_to_fC_x, 0, length_rC_to_fC_z);

      // Trans Matrix from center of the robot to  Foot Plate center
       // The horizontal length between robot center and Foot Plate center
       double length_rC_to_tC_x = length_rC_to_fC_x + conf.dyna.length_from_axis_to_tip;
       double length_rC_to_tC_z =  conf.jLength.length_CTJ_to_FTJ + (-conf.foot.length + conf.dyna.width)/2.;
       // Matrix
       osg::Matrix trans_rC_to_tC =  Matrix::translate( length_rC_to_tC_x, 0, length_rC_to_tC_z);

      // Trans Matrix from center of the robot to center of the foot sphere
       // Matrix
       osg::Matrix trans_rC_to_fsC =  Matrix::translate( length_rC_to_tC_x, 0, length_rC_to_tC_z - conf.foot.length/2. - conf.foot.footRadius  );

       // Trans Matrix from center of the robot to center of the foot sphere
        // Matrix
       osg::Matrix trans_tC_to_fsC =  Matrix::translate( 0, 0,  - conf.foot.length/2. - conf.foot.footRadius);


      // Trans Matrix from center of the robot to TC joint Center
       // Matrix
       osg::Matrix trans_rC_to_TCj = Matrix::translate( conf.jLength.length_center_to_TCJ, 0, 0);

      // Trans Matrix from center of the robot to CT joint Center
       // Matrix
       osg::Matrix trans_rC_to_CTj = Matrix::translate( conf.jLength.length_center_to_TCJ + conf.jLength.length_TCJ_to_CTJ, 0, 0);

      // Trans Matrix from center of the robot to FT joint Center
       // Matrix
       osg::Matrix trans_rC_to_FTj = Matrix::translate( conf.jLength.length_center_to_TCJ + conf.jLength.length_TCJ_to_CTJ, 0, + conf.jLength.length_CTJ_to_FTJ);


    for(int i = 0; i < LEG_POS_MAX; i++){
            // Name of the Leg
            LegPos leg = LegPos(i);

            // Rotation Matrix : change the rotation direction according to leg number
            // By using this, the direction of leg can be changed.. It is used to adapt to the location of the leg
            Matrix legRotate;
            if(leg == L0){
                    legRotate = Matrix::rotate(0, Vec3(0, 0, 1));
            }else if(leg == L1){
                    legRotate = Matrix::rotate(M_PI*2./3., Vec3(0, 0, 1));
            }else{
                    legRotate = Matrix::rotate(-M_PI*2./3., Vec3(0, 0, 1));
            }

      // Shoulder ******************************************
      //  We connect the sholder dynamixel to main Body
#ifdef DEBUG_MODE
        cout << "@@@@ Creation of Shoulder starts!! @@@@" << endl;
#endif

             // sholder Dynamixel Box
            Box* dBox =  new Box( conf.dyna.length, conf.dyna.width, conf.dyna.height);
            dBox->getOSGPrimitive()->setTexture("Images/wood.rgb");
            //dBox->setMass(conf.dyna.mass);
             // trans (locate the object on the hexagon plane)
             //  first, I turn the object to leg's direction and translate it to desired position ()
             // mention that this pose matrix is multipled vector from right hand <- it is different from usual one!!
            ImpTransform* dTrans = new ImpTransform(trunk.tPlate[0], dBox,
                        trans_rC_to_sC * trans_rO_to_rC * legRotate);
                dTrans->init(odeHandle, conf.dyna.mass, dynaHandle);

            /*
            //Debug Mode
            if(leg == L1 || leg == L2){
                    dTrans->init(odeHandle, 1., dynaHandle);
            }else{
                    dTrans->init(odeHandle, conf.dyna.mass, dynaHandle);
            }
            */


                 // save to the leg struct
                legs[leg].shoulderBox = dBox;
                legs[leg].shoulder = dTrans;
                // add it to object
                objects.push_back(legs[leg].shoulder);

         // Coxa **********************************************
         // We make the first joint and link of Ashigaru
#ifdef DEBUG_MODE
        cout << "@@@@ Creation of Coxa starts!! @@@@" << endl;
#endif

           // Make the Dyanmixel for link
                 // Coxa (1st link) Dynamixel Box
                Box* link1dBox = new Box( conf.dyna.length, conf.dyna.width, conf.dyna.height);
                link1dBox->getOSGPrimitive()->setTexture("Images/wood.rgb");
                link1dBox->init(odeHandle, conf.dyna.mass , dynaHandle);
// about the pose or something
                // set the pose
                 //  pose:: pose represent the attitude and position of the object by quotanion
                 //         we can use the function rotate and translate as if it is attitude change 4*4 matrix
                 //
                 // I should say about mechanism of the pose in my understanding.
                 //   Pose ; this is the 4 by 4 matrix and it represents the rotation and translation
                 //          It can be used as if it is normal 4 by 4 rotation matrix.
                 //          But what you should notice is that this matrix is for Row Vector (1 by 4 vector)
                 //          So, the row of the calculating is inverse to normal one!!
                 //                         You should be careful about it

                Matrix l1Pose = Matrix::rotate( M_PI / 2., Vec3(1, 0, 0)) * /* this rotation is used to turn the dymamixel to optimal attitude!! (because, we change the axis of joints) */
                                trans_rC_to_cC * trans_rO_to_rC * legRotate * pose;  //  first, I turn the object to leg's direction and translate it to desired position ()
                link1dBox->setPose(l1Pose);
                 // save it to leg struct
                legs[leg].coxa = link1dBox;
                 // add it to object
                objects.push_back(legs[leg].coxa);

          // TC joints *******************************:::::::
           // create the joint of dynamixel on Body (from 1st link to body)
                 // calculate the joint pose (attitude and position) by using trans and rotate function
                osg::Matrix j1Pose = trans_rC_to_TCj * trans_rO_to_rC * legRotate * pose;
                 // To make a joint, we need the position vector only, so multiple nullpos to the pose to get the vector.
                 //  the attitude is determined by axis (it is z axis)
                HingeJoint* TCj = new HingeJoint( trunk.tPlate[0], link1dBox,  nullpos * j1Pose, Axis(0,0,1) * legRotate );
                // ^ Notice: When we want to rotate the Axis() like Pose, we have to multiple the rotation matrix from RIGHT SIDE
                //     Not Left side, it is very different from real calculation
                TCj->init(odeHandle, osgHandle, true, conf.rate * 0.04);
                 // add the joint to the joint vector
                joints.push_back(TCj);

                 // create motor (by using servo motor, we will do position control)
                 //OneAxisServo* servo1 = new OneAxisServoVel(odeHandle, TCj, conf.servoParam.TC_angle_MIN, conf.servoParam.TC_angle_MAX, conf.servoParam.power, conf.servoParam.damp, conf.servoParam.maxVel, 1.0);
         //Debug
                OneAxisServo* servo1;
                if(i == conf.specialParam.conectedLegNum){
                        servo1 = new OneAxisServoVel(odeHandle, TCj, conf.servoParam.TC_angle_MIN, conf.servoParam.TC_angle_MAX, conf.specialParam.servoPower, conf.servoParam.damp, conf.servoParam.maxVel, 1.0);
                }else servo1 = new OneAxisServoVel(odeHandle, TCj, conf.servoParam.TC_angle_MIN, conf.servoParam.TC_angle_MAX, conf.servoParam.power, conf.servoParam.damp, conf.servoParam.maxVel, 1.0);

                 // save it to leg struct
                legs[leg].tcJoint = TCj;
                legs[leg].tcServo = servo1;
                 // add it to servo Map
                servos[getMotorName(leg, TC)] = servo1;

                 // create torque sensor
                  // In this time, I do not use scaller just get real value
                motorTorqSensors[getMotorName(leg, TC)] = new TorqueSensor(1.);
                  // initialize (own is Null, it was not used in simulation)
                motorTorqSensors[getMotorName(leg, TC)]->init(NULL, TCj);


         // Femur **********************************************
         // We make the second joint and link of Ashigaru (CT joint and femur)
#ifdef DEBUG_MODE
        cout << "@@@@ Creation of Femur starts!! @@@@" << endl;
#endif

           // Make the Dyanmixel for link
                 // Femur (2nd link) Dynamixel Box
                Box* link2dBox = new Box( conf.dyna.length, conf.dyna.width, conf.dyna.height);
                link2dBox->getOSGPrimitive()->setTexture("Images/wood.rgb");
                link2dBox->init(odeHandle, conf.dyna.mass , dynaHandle);
                 // set the pose
                 //  pose:: pose represent the attitude and position of the object by quotanion
                 //         we can use the function rotate and translate as if it is attitude change 4*4 matrix
                osg::Matrix l2Pose = Matrix::rotate( M_PI / 2., Vec3(0, 0, 1)) * Matrix::rotate( M_PI / 2., Vec3(1, 0, 0)) * // this rotation is used to turn the dymamixel to optimal attitude!! (because, we change the axis of joints)
                                  trans_rC_to_fC * trans_rO_to_rC * legRotate * pose;  //  first, I turn the object to leg's direction and translate it to desired position ()
                link2dBox->setPose(l2Pose);
                 // save it to leg struct
                legs[leg].femur = link2dBox;
                 // add it to object
                objects.push_back(legs[leg].femur);

         // CT joint :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
           // create the joint of dynamixel on Body (from 2nd link to 1st Link)
                 // calculate the joint pose (attitude and position) by using trans and rotate function
                osg::Matrix j2Pose = trans_rC_to_CTj * trans_rO_to_rC * legRotate * pose;
                 // To make a joint, we need the position vector only, so multiple nullpos to the pose to get the vector.
                 //  the attitude is determined by axis (it is z axis)
                HingeJoint* CTj = new HingeJoint( legs[leg].coxa, link2dBox, nullpos * j2Pose,  Axis(0,1,0) * legRotate);
                // ^ Notice: When we want to rotate the Axis() like Pose, we have to multiple the rotation matrix from RIGHT SIDE
                //     Not Left side, it is very different from real calculation
                CTj->init(odeHandle, osgHandle, true, conf.rate * 0.04);
                 // add the joint to the joint vector
                joints.push_back(CTj);

                 // create motor (by using servo motor, we will do position control)
                 // OneAxisServo* servo2 = new OneAxisServoVel(odeHandle, CTj, conf.servoParam.CT_angle_MIN, conf.servoParam.CT_angle_MAX, conf.servoParam.power, conf.servoParam.damp, conf.servoParam.maxVel, 1.0);
                //Debug
                OneAxisServo* servo2;
                if( i == conf.specialParam.conectedLegNum){
                        servo2 = new OneAxisServoVel(odeHandle, CTj, conf.servoParam.CT_angle_MIN, conf.servoParam.CT_angle_MAX, conf.specialParam.servoPower, conf.servoParam.damp, conf.servoParam.maxVel, 1.0);
                }else servo2 = new OneAxisServoVel(odeHandle, CTj, conf.servoParam.CT_angle_MIN, conf.servoParam.CT_angle_MAX, conf.servoParam.power, conf.servoParam.damp, conf.servoParam.maxVel, 1.0);

                 // save it to leg struct
                legs[leg].ctJoint = CTj;
                legs[leg].ctServo = servo2;
                 // add it to servo Map
                servos[Ashigaru::getMotorName(leg, CT)] = servo2;

                 // create torque sensor
                  // In this time, I do not use scaller just get real value
                motorTorqSensors[getMotorName(leg, CT)] = new TorqueSensor(1.);
                  // initialize (own is Null, it was not used in simulation)
                motorTorqSensors[getMotorName(leg, CT)]->init(NULL, CTj);

         // Tibia **********************************************
         // We make the third joint and link of Ashigaru (FT joint and femur)
#ifdef DEBUG_MODE
        cout << "@@@@ Creation of Tibia starts!! @@@@" << endl;
#endif

           // Make the Plate
                 // Tibia (3rd link) Plate
                Box* link3dBox = new Box( conf.foot.height, conf.foot.width, conf.foot.length);
                link3dBox->getOSGPrimitive()->setTexture("Images/wood.rgb");
                link3dBox->init(odeHandle, conf.foot.mass , tibiaHandle[leg]);
                 // set the pose
                 //  pose:: pose represent the attitude and position of the object by quotanion
                 //         we can use the function rotate and translate as if it is attitude change 4*4 matrix
                osg::Matrix l3Pose = trans_rC_to_tC * trans_rO_to_rC * legRotate * pose;  //  first, I turn the object to leg's direction and translate it to desired position ()
                link3dBox->setPose(l3Pose);
                 // save it to leg struct
                legs[leg].tibia = link3dBox;
                 // add it to object
                objects.push_back(legs[leg].tibia);


           //Make Foot
                 // Sphere
                Sphere* foot = new Sphere( conf.foot.footRadius );
                foot->getOSGPrimitive()->setTexture("Images/wood.rgb");
                 // Set the substance of foot
                OdeHandle rubberHandle(odeHandle);
        const Substance FootSubstance(3.0, 0.0, 500.0, 0.1);
        rubberHandle.substance = FootSubstance;

        legs[leg].footSphere = foot;
         //translate
        legs[leg].foot = new ImpTransform(legs[leg].tibia, foot, trans_tC_to_fsC);
         // initialize
        legs[leg].foot->init(rubberHandle, 0.0, osgHandle);
                  //add it to objects
                  objects.push_back(legs[leg].foot);

         // FT joint :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
           // create the joint of dynamixel on Femur (from 3rd link to 2nd Link)
                 // calculate the joint pose (attitude and position) by using trans and rotate function
                osg::Matrix j3Pose = trans_rC_to_FTj * trans_rO_to_rC * legRotate * pose;
                 // To make a joint, we need the position vector only, so multiple nullpos to the pose to get the vector.
                 //  the attitude is determined by axis (it is z axis)
                HingeJoint* FTj = new HingeJoint( legs[leg].femur, link3dBox, nullpos * j3Pose, Axis(0,1,0) * legRotate);
                // ^ Notice: When we want to rotate the Axis() like Pose, we have to multiple the rotation matrix from RIGHT SIDE
                //     Not Left side, it is very different from real calculation
                FTj->init(odeHandle, osgHandle, true, conf.rate * 0.04);
                 // add the joint to the joint vector
                joints.push_back(FTj);

                 // create motor (by using servo motor, we will do position control)
                 // OneAxisServo* servo3 = new OneAxisServoVel(odeHandle, FTj, conf.servoParam.FT_angle_MIN, conf.servoParam.FT_angle_MAX, conf.servoParam.power, conf.servoParam.damp, conf.servoParam.maxVel, 1.0);
                //Debug
                OneAxisServo* servo3;
                if(i == conf.specialParam.conectedLegNum){
                        servo3 = new OneAxisServoVel(odeHandle, FTj, conf.servoParam.FT_angle_MIN, conf.servoParam.FT_angle_MAX, conf.specialParam.servoPower, conf.servoParam.damp, conf.servoParam.maxVel, 1.0);
                }else servo3 = new OneAxisServoVel(odeHandle, FTj, conf.servoParam.FT_angle_MIN, conf.servoParam.FT_angle_MAX, conf.servoParam.power, conf.servoParam.damp, conf.servoParam.maxVel, 1.0);

                 // save it to leg struct
                legs[leg].ftJoint = FTj;
                legs[leg].ftServo = servo3;
                 // add it to servo Map
                servos[Ashigaru::getMotorName(leg, FT)] = servo3;

                 // create torque sensor
                  // In this time, I do not use scaller just get real value
                motorTorqSensors[getMotorName(leg, FT)] = new TorqueSensor(1.);
                  // initialize (own is Null, it was not used in simulation)
                motorTorqSensors[getMotorName(leg, FT)]->init(NULL, FTj);

                // Leg contact Sensors (Foot toe)
                legContactSensors[leg] = new ContactSensor(false, 100, conf.foot.footRadius*1.1, true);//make the sphere a little bit larger than real foot to detect touching
                 // We set the contact sensor at the point of foot sphere
                legContactSensors[leg]->setInitData(odeHandle, osgHandle, trans_tC_to_fsC);
                legContactSensors[leg]->init(legs[leg].tibia);
                //odeHandle.addIgnoredPair(legs[leg].foot, legContactSensors[leg]->getTransformObject());
    }




#ifdef DEBUG_MODE
    cout << "@@@@ Creation Phase ends!! @@@@" << endl;
#endif

    created=true;
  };

  // getTorqueSensorData
  sensor Ashigaru::getTorqueData(TorqueSensor*  torqueSensor){
    // Georg: the following should do:
    if(torqueSensor)
      return torqueSensor->getList().front();
    else
      return -1.;
                /*
                //
          std::list<sensor> a;
          std::list<sensor>::iterator it;
          if(torqueSensor){
                 a = torqueSensor->getList();
                 it = a.end();
                 --it;
                 //std::cout << "trq " << *it << std::endl;
                 return *it;
                 }else  return -1.;
                */
  }

  // convert Pose Matrix(Quatanion) to the roll, pitch, yaw angle (rad)
  osg::Vec3d Ashigaru::convert_Quat_to_RollPitchYaw(osg::Quat quat){
          osg::Vec3d rpy;
          // the 3 vetors of Matrix
          osg::Vec3d M0, M1, M2;

          // use the quaternion to rotate this vector and get Matrix elements
          M0 = quat * osg::Vec3(1., 0, 0);
          M1 = quat * osg::Vec3(0, 1., 0);
          M2 = quat * osg::Vec3(0, 0, 1.);

          // calc roll pitch yaw
          rpy[2] = atan2( M0[1], M0[0]); // yaw
          rpy[1] = atan2( - M0[2], sqrt(M1[2]*M1[2] +  M2[2]*M2[2]) );  // pitch
          rpy[0] = atan2( M1[2], M2[2]); // roll
          return rpy;
  }

  // calculate COG Position
  osg::Vec3d Ashigaru::calc_COGPosition(void){
          // cogPos
          osg::Vec3d cogPos;

          // add many objects but I do it manually because many types of objects we have
           // Trunk lower hexagonal plate
           cogPos = this->getMainPrimitive()->getPosition() * (conf.body.mass / 2.);
           // Trunk higher hexagonal plate
           cogPos += trunk.tUpTrans->getChildPose().getTrans() * (conf.body.mass / 2.);

           // Leg clculation
           for(int i = 0; i < LEG_POS_MAX; i++){
                    // Name of the Leg
                    LegPos leg = LegPos(i);
                    // shoulder
                    cogPos += legs[leg].shoulder->getChildPose().getTrans() * conf.dyna.mass;
                    // coxa
                    cogPos += legs[leg].coxa->getPosition() * conf.dyna.mass;
                    // femur
                    cogPos += legs[leg].femur->getPosition() * conf.dyna.mass;
                    // tibia plate
                    cogPos += legs[leg].tibia->getPosition() * conf.foot.mass;
                    // foot sphere has no weight!!
           }

           cogPos = cogPos / conf.wholeMass;
           return cogPos;
           // calc COG
  }

  /** destroys ASHIGARU and space
   */
  void Ashigaru::destroy(){
    if (created){
      //irSensorBank.clear();
      cleanup();
      odeHandle.deleteSpace();

      // delete contact sensors
      for (int i = 0; i < LEG_POS_MAX; i++) {
        if (legContactSensors[LegPos(i)])
          delete legContactSensors[LegPos(i)];
      }
      legContactSensors.clear();
    }
    created=false;
  }

  // constructor of struct Leg
  Ashigaru::Leg::Leg():
          tcJoint(0), ctJoint(0), ftJoint(0), footJoint(0),
          tcServo(0), ctServo(0), ftServo(0),
          footSpring(0), shoulder(0), shoulderBox(0),
          coxa(0),femur(0),tibia(0),foot(0){
  }

  // constructor of struct trunk
  Ashigaru::Trunk::Trunk():
    tPlate{0},tTrans{0}{
  }

}

