/*
 * runbot.cpp
 *
 *  Created on: 06.11.2013
 *      Author: Johannes Widenka
 */

#include <assert.h>
#include "runbotii.h"

//Using namespaces
using namespace osg;
using namespace std;
namespace lpzrobots {

  Runbot::Runbot(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
      const RunbotConf& conf, const string& name)
  : OdeRobot(odeHandle, osgHandle, name, "revision 1.0"), conf(conf){

  }

  Runbot::~Runbot(){

  }

  void Runbot::placeIntern(const Matrix& pose){
    //TODO:Configuration checks
    Matrix initialPose;
    //no initialization of the pose necessary at the moment
    initialPose = Matrix::identity()*pose;
    //creating the Robot
    create(initialPose);
  }

  void Runbot::create(const Matrix& pose){
    Matrix tempPose;
    double s = conf.scale;
    Matrix sideTranslation = Matrix::identity();
    if (conf.armLength > 0)
      sideTranslation = Matrix::translate(-(conf.bodyWidth*s/2)-conf.armLength*s,-conf.armOffset*s,0);
    /**
     * left side creation..
     */
    //foot
    double tempHeight = conf.footHeight*s/2;
    double translation = -conf.legDistFromCenter;
    Primitive* lFoot = new Box(conf.footWidth*s,conf.footLength*s,conf.footHeight*s);
    lFoot->init(odeHandle,conf.footMass,osgHandle.changeColor(Color(0.9,0.9,0.9)));
    tempPose = Matrix::translate(translation*s,conf.footLength*s/2-conf.footAnklePos*s,tempHeight);
    lFoot->setPose(tempPose*sideTranslation*pose);
    tempHeight = conf.footHeight*s;

    //lower leg
    tempHeight -= conf.legAnklePos*s;
    Primitive* lLowLeg = new Box(conf.lowerLegThickness*s,conf.lowerLegThickness*s,conf.lowerLegLength*s);
    lLowLeg->init(odeHandle,conf.lowerLegMass,osgHandle.changeColor(Color(0.7,0.7,0.7)));
    tempPose = Matrix::translate(translation*s,0,conf.lowerLegLength*s/2+tempHeight);
    lLowLeg->setPose(tempPose*sideTranslation*pose);


    //upper leg
    tempHeight += conf.lowerKneePos*s-conf.upperKneePos*s;
    Primitive* lUpLeg = new Box(conf.upperLegThickness*s,conf.upperLegThickness*s,conf.upperLegLength*s);
    lUpLeg->setColor(lpzrobots::Color(0.2,0.2,0.2,1));
    lUpLeg->init(odeHandle,conf.upperLegMass,osgHandle.changeColor(Color(0.8,0.8,0.8)));
    tempPose = Matrix::translate(translation*s,0,conf.upperLegLength*s/2+tempHeight);
    lUpLeg->setPose(tempPose*sideTranslation*pose);

    /**
     * right side creation..
     */
    //foot
    tempHeight = conf.footHeight*s/2;
    translation = +conf.legDistFromCenter;
    Primitive* rFoot = new Box(conf.footWidth*s,conf.footLength*s,conf.footHeight*s);
    rFoot->init(odeHandle,conf.footMass,osgHandle.changeColor(Color(0.9,0.9,0.9)));
    tempPose = Matrix::translate(translation*s,conf.footLength*s/2-conf.footAnklePos*s,tempHeight);
    rFoot->setPose(tempPose*sideTranslation*pose);
    tempHeight = conf.footHeight*s;

    //lower leg
    tempHeight -= conf.legAnklePos*s;
    Primitive* rLowLeg = new Box(conf.lowerLegThickness*s,conf.lowerLegThickness*s,conf.lowerLegLength*s);
    rLowLeg->init(odeHandle,conf.lowerLegMass,osgHandle.changeColor(Color(0.7,0.7,0.7)));
    tempPose = Matrix::translate(translation*s,0,conf.lowerLegLength*s/2+tempHeight);
    rLowLeg->setPose(tempPose*sideTranslation*pose);


    //upper leg
    tempHeight += conf.lowerKneePos*s-conf.upperKneePos*s;
    Primitive* rUpLeg = new Box(conf.upperLegThickness*s,conf.upperLegThickness*s,conf.upperLegLength*s);
    rUpLeg->init(odeHandle,conf.upperLegMass,osgHandle.changeColor(Color(0.8,0.8,0.8)));
    tempPose = Matrix::translate(translation*s,0,conf.upperLegLength*s/2+tempHeight);
    rUpLeg->setPose(tempPose*sideTranslation*pose);

    printf("bodycreation\n");
    /**
     * Center creation..
     */
    //creating the body
    tempHeight += conf.legHipPos*s-conf.bodyHipHeight*s;
    Primitive* body = new Box(conf.bodyWidth*s,conf.bodyDepth*s,conf.bodyHeight*s);
    body->init(odeHandle, conf.bodyMass,osgHandle.changeColor(Color(0.1,0.8,0.5)));
    tempPose = Matrix::translate(0,conf.bodyDepth*s/2-conf.bodyHipDepth*s,conf.bodyHeight*s/2+tempHeight);
    body->setPose(tempPose*sideTranslation*pose);

    //creating the movable weight
    tempHeight += conf.massjointBodyHeight*s-conf.massjointMassHeight*s;
    Primitive* weight = new Box(conf.weightWidth*s,conf.weightDepth*s,conf.weightHeight*s);
    weight->setColor("black");
    weight->init(odeHandle, conf.bodyMass,osgHandle.changeColor(Color(0.2,0.3,1.0)));
    tempPose = Matrix::translate(0,-conf.bodyHipDepth*s+(conf.weightDepth)*s/2,conf.weightHeight*s/2+tempHeight);
    weight->setPose(tempPose*sideTranslation*pose);





    //move the walking part of the robot to the side

    objects.push_back(body);
    objects.push_back(lFoot);
    int lFootIdx = objects.size()-1;
    objects.push_back(lLowLeg);
    objects.push_back(lUpLeg);
    objects.push_back(rFoot);
    int rFootIdx = objects.size()-1;
    objects.push_back(rLowLeg);
    objects.push_back(rUpLeg);
    objects.push_back(weight);


    /**
     * creating joints
     */

    HingeJoint* lAnkle = new HingeJoint(lFoot,lLowLeg,lFoot->getPosition()+Vec3d(0,-(conf.footLength*s/2)+conf.footAnklePos*s,0),Axis(1,0,0)*lFoot->getPose());
    HingeJoint* rAnkle = new HingeJoint(rFoot,rLowLeg,rFoot->getPosition()+Vec3d(0,-(conf.footLength*s/2)+conf.footAnklePos*s,0),Axis(1,0,0)*rFoot->getPose());
    HingeJoint* lKnee = new HingeJoint(lLowLeg,lUpLeg,lLowLeg->getPosition()+Vec3d(0,0,-(conf.lowerLegLength*s/2)+conf.lowerKneePos*s),Axis(1,0,0)*lLowLeg->getPose());
    HingeJoint* rKnee = new HingeJoint(rLowLeg,rUpLeg,rLowLeg->getPosition()+Vec3d(0,0,-(conf.lowerLegLength*s/2)+conf.lowerKneePos*s),Axis(1,0,0)*rLowLeg->getPose());
    HingeJoint* lHip = new HingeJoint(lUpLeg,body,lUpLeg->getPosition()+Vec3d(0,0,-(conf.upperLegLength*s/2)+conf.legHipPos*s),Axis(1,0,0)*lUpLeg->getPose());
    HingeJoint* rHip = new HingeJoint(rUpLeg,body,rUpLeg->getPosition()+Vec3d(0,0,-(conf.upperLegLength*s/2)+conf.legHipPos*s),Axis(1,0,0)*rUpLeg->getPose());
    HingeJoint* massJoint = new HingeJoint(body,weight,body->getPosition()+Vec3d(0,-(conf.bodyDepth*s/2)+conf.massjointBodyDepth*s,-(conf.bodyHeight*s/2)+conf.massjointBodyHeight*s),Axis(1,0,0)*body->getPose());

    lAnkle->init(odeHandle,osgHandle,true,2.5*s);
    joints.push_back(lAnkle);
    rAnkle->init(odeHandle,osgHandle,true,2.5*s);
    joints.push_back(rAnkle);
    lKnee->init(odeHandle,osgHandle,true,2.5*s);
    joints.push_back(lKnee);
    rKnee->init(odeHandle,osgHandle,true,2.5*s);
    joints.push_back(rKnee);
    lHip->init(odeHandle,osgHandle,true,2.5*s);
    joints.push_back(lHip);
    rHip->init(odeHandle,osgHandle,true,2.5*s);
    joints.push_back(rHip);
    massJoint->init(odeHandle,osgHandle,true,2.5*s);
    joints.push_back(massJoint);

    //creating the center and the arm
    if (conf.armLength > 0){
      double shoulderHeight = tempHeight;
      Primitive* center = new Cylinder(conf.armLength*s/3,1*s);
      center->init(odeHandle,20,osgHandle.changeColor(0.5,0.0,0.5)); //standard 'high' mass (fixed)
      tempPose = Matrix::translate(0,0,s/2);
      center->setPose(tempPose*pose);
      centerAnchor = center;

      Primitive* arm = new Box(conf.armLength*s,1*s,1*s);
      arm->setColor(lpzrobots::Color(1.0,1.0,0.0));
      arm->init(odeHandle,0.2,osgHandle);
      tempPose = Matrix::translate(-conf.armLength*s/2,0,shoulderHeight);
      arm->setPose(tempPose*pose);
      objects.push_back(arm);
      objects.push_back(center);

      Hinge2Joint* centerJoint = new Hinge2Joint(center,arm,center->getPosition()+Vec3d(0,0,-(1*s/2)+shoulderHeight),Axis(0,0,1)*arm->getPose(),Axis(0,1,0)*arm->getPose());
      centerJoint->init(odeHandle,osgHandle,true,1*s);
      joints.push_back(centerJoint);
      this->centerJoint = centerJoint;

      HingeJoint* shoulder = new HingeJoint(body,arm,arm->getPosition()+Vec3d(-conf.armLength*s/2,0,0),Axis(1,0,0)*body->getPose());
      shoulder->init(odeHandle,osgHandle,true,1*s);
      joints.push_back(shoulder);

      printf("body creation done\n");

    }


    //----Creating the motors-----//
    OneAxisServo* lHipMotor = new OneAxisServoVelDisconnected(odeHandle,lHip,conf.hipMotorMin,conf.hipMotorMax,conf.hipMotorPower,0.95,conf.hipMotorMaxVel);
    motors.push_back(lHipMotor);

    OneAxisServo* rHipMotor = new OneAxisServoVelDisconnected(odeHandle,rHip,conf.hipMotorMin,conf.hipMotorMax,conf.hipMotorPower,0.95,conf.hipMotorMaxVel);
    motors.push_back(rHipMotor);

    OneAxisServo* lKneeMotor = new OneAxisServoVelDisconnected(odeHandle,lKnee,conf.kneeMotorMin,conf.kneeMotorMax,conf.kneeMotorPower,0.95,conf.kneeMotorMaxVel);
    motors.push_back(lKneeMotor);

    OneAxisServo* rKneeMotor = new OneAxisServoVelDisconnected(odeHandle,rKnee,conf.kneeMotorMin,conf.kneeMotorMax,conf.kneeMotorPower,0.95,conf.kneeMotorMaxVel);
    motors.push_back(rKneeMotor);

    OneAxisServo* massMotor = new OneAxisServoVel(odeHandle,massJoint,conf.massMotorMin,conf.massMotorMax,conf.massMotorPower,0.05,conf.massMotorMaxVel);
    motors.push_back(massMotor);

    OneAxisServo* lFootAnkleSpring = new OneAxisServoVel(odeHandle,lAnkle,conf.ankleMotorMin,conf.ankleMotorMax,conf.ankleMotorPower,conf.ankleMotorDamping);
    passiveMotors.push_back(lFootAnkleSpring);

    OneAxisServo* rFootAnkleSpring = new OneAxisServoVel(odeHandle,rAnkle,conf.ankleMotorMin,conf.ankleMotorMax,conf.ankleMotorPower,conf.ankleMotorDamping);
    passiveMotors.push_back(rFootAnkleSpring);


    /*
     * coloring the foot sensors not possible at the moment, since the feet are
     * of Primitive type "Box", which encapsulates OSGBoxText, where setColor()
     * is not implemented at the moment..
     * For visual feedback, activate the collision sphere. The downside is, that
     * collision detection is then based on the sphere as well.
     */
    //----Creating the sensors----//
    ContactSensor* lContact = new ContactSensor(true,1,conf.scale*1,false,false); //binary contactsensors
    lContact->setInitData(odeHandle,osgHandle,osg::Matrix::translate(.0,.0,-conf.scale*conf.footHeight/2.0));
    addSensor(std::shared_ptr<Sensor>(lContact),Attachment(lFootIdx));
    contactsensors.push_back(lContact);
    //odeHandle.addIgnoredPair(lLowLeg, lContact->getTransformObject());

    ContactSensor* rContact = new ContactSensor(true,1,conf.scale*1,false,false);
    rContact->setInitData(odeHandle,osgHandle,osg::Matrix::translate(.0,.0,-conf.scale*conf.footHeight/2.0));
    addSensor(std::shared_ptr<Sensor>(rContact),Attachment(rFootIdx));
    contactsensors.push_back(rContact);
    //odeHandle.addIgnoredPair(rLowLeg, rContact->getTransformObject());


  }

  int Runbot::getSensorsIntern(sensor* sensors, int sensorNumber){
    double RadtoDeg = 360.0/(2*M_PI);
    int n = 0;
    double hipAngleOffset = M_PI/2.0; //the angle from the base of measurement to the base pose  (RAD)
    double kneeAngleOffset = M_PI;    // ''
    sensors[0] = (motors[0]->get()*0.5*(conf.hipMotorMax-conf.hipMotorMin)+hipAngleOffset + (conf.hipMotorMax + conf.hipMotorMin)/2)*RadtoDeg;
    n++;
    sensors[1] = (motors[1]->get()*0.5*(conf.hipMotorMax-conf.hipMotorMin)+hipAngleOffset + (conf.hipMotorMax + conf.hipMotorMin)/2)*RadtoDeg;
    n++;
    sensors[2] = (motors[2]->get()*0.5*(conf.kneeMotorMax-conf.kneeMotorMin)+kneeAngleOffset + (conf.kneeMotorMax + conf.kneeMotorMin)/2)*RadtoDeg;
    n++;
    sensors[3] = (motors[3]->get()*0.5*(conf.kneeMotorMax-conf.kneeMotorMin)+kneeAngleOffset + (conf.kneeMotorMax + conf.kneeMotorMin)/2)*RadtoDeg;
    n++;
    sensors[4] = motors[4]->get();
    n++;
    sensors[5] = 4096-contactsensors[0]->get()*2048;
    n++;
    sensors[6] = 4096-contactsensors[1]->get()*2048;
    n++;
    sensors[7] = centerJoint->getPosition1()*10; //scaled for less noise in speed calculation
    n++;
    return n;
  }


  //realrunbots gets its motorvalues in -5V..+5V
  void Runbot::setMotorsIntern(const motor* motors, int motorNumber){
    assert(motorNumber == getMotorNumber());
    int inv = 1;
    if (conf.invertMotors)
      inv = -1;
    double inputFactor = 1.0/4.8;
    this->motors[0]->set(motors[0]*inputFactor);
    this->motors[1]->set(motors[1]*inputFactor);
    this->motors[2]->set(motors[2]*inputFactor);
    this->motors[3]->set(motors[3]*inputFactor);
    this->motors[4]->set(motors[4]);
  }

  int Runbot::getSensorNumberIntern(){
    int n = 0;
    n += motors.size();
    n += contactsensors.size();
    n++;
    return n;
  }

  int Runbot::getMotorNumberIntern(){
    return motors.size();
  }


  void Runbot::doInternalStuff(GlobalData& globalData){

	OdeRobot::doInternalStuff(globalData);
    //ankle springs
    passiveMotors[0]->set(0);
    passiveMotors[1]->set(0);


  }

  Primitive* Runbot::getCenterAnchor(){
    return centerAnchor;
  }

  double Runbot::sign(double v)
  {
    if(v > 0.0)
      return 1.0;
    else if(v < 0.0)
      return -1.0;
    else
      return 0.0;
  }

}// end of namespace lpzrobots
