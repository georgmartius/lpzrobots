#include "differential.h"

// Using namespaces
using namespace osg;
using namespace std;
namespace lpzrobots{

  Differential::Differential(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
      const DifferentialConf& conf, const string& name)
    : OdeRobot(odeHandle, osgHandle, name, "revision 1.0"), conf(conf){
    }


  Differential::~Differential(){
  }


  void Differential::place(const Matrix& pose){
    // Movig robot upward so wheel are not stuck on the ground
    Matrix initialPose;
    initialPose = Matrix::translate(Vec3(0, 0, conf.wheelRadio) * pose);
    // Creating the robot
    create(initialPose);
  }


  void Differential::create(const Matrix& pose) {
    /* Creating body */
    // Cylinder geometry primitive as body
    Primitive* body = new Cylinder(conf.bodyRadio, conf.bodyHeight);
    // Setting texture from Image library
    body->setTexture("Images/purple_velour.jpg");
    // Initializing the primitive
    body->init(odeHandle, conf.bodyMass, osgHandle);
    // Setting the pose of the primitive
    body->setPose(pose);
    // Adding the primitive to the list of objects
    objects.push_back(body);

    /* Creating the left wheel */
    Primitive* lWheel = new Cylinder(conf.wheelRadio, conf.wheelHeight);
    // Setting texture from Images library
    lWheel->setTexture("Images/chess.rgb");
    lWheel->init(odeHandle, conf.wheelMass, osgHandle);
    // The cylinder is rotated 90º on the Y axis
    // then translated outside the radio of the body plus half of 
    // its own height
    // -- All transformations have to be relative to the position so 
    // the robot can be initialised in any position --
    Matrix lWheelPose = Matrix::rotate(M_PI / 2.0, 0, 1, 0) * 
      Matrix::translate(conf.bodyRadio + conf.wheelHeight / 2.0, .0, .0) *
      pose;
    // Setting the pose of the wheel
    lWheel->setPose(lWheelPose);
    // adding the wheel to the list of objects
    objects.push_back(lWheel);
    // Joining the wheel to the body by a hingejoint
    // objecst[0] is the body, objects[1] is the left wheel
    // the anchor comes from the wheel and the axis of rotation
    // is relative to the pose of the left wheel
    HingeJoint* bodyLeftWheelJoint = new HingeJoint(objects[0], objects[1],
        lWheel->getPosition(),
        Axis(0, 0, 1) * lWheelPose);
    // Initializing the joint
    bodyLeftWheelJoint->init(odeHandle, osgHandle);
    // Adding the joint to the list of joints
    joints.push_back(bodyLeftWheelJoint);

    /* Creating the right wheel */
    // Analog to left wheel but changing translation direction
    Primitive* rWheel = new Cylinder(conf.wheelRadio, conf.wheelHeight);
    rWheel->setTexture("Images/chess.rgb");
    rWheel->init(odeHandle, conf.wheelMass, osgHandle);
    Matrix rWheelPose = Matrix::rotate(M_PI / 2.0, 0, 1, 0) * 
      Matrix::translate(-(conf.bodyRadio + conf.wheelHeight / 2.0), .0, .0) *
      pose;
    rWheel->setPose(rWheelPose);
    objects.push_back(rWheel);
    // objects[2] is the right wheel
    HingeJoint* bodyRightWheelJoint = new HingeJoint(objects[0], objects[2],
        rWheel->getPosition(),
        Axis(0, 0, 1) * rWheelPose);
    bodyRightWheelJoint->init(odeHandle, osgHandle);
    joints.push_back(bodyRightWheelJoint);

    /* Motors */
    // Left wheel motor, the OdeHandle, the joint and the maximun 
    // power that motor will be used to achieve desired speed
    AngularMotor1Axis* lWheelMotor = new AngularMotor1Axis(odeHandle, bodyLeftWheelJoint,
      conf.wheelMotorPower);
    wheelMotors.push_back(lWheelMotor);
    // Right wheel motor
    AngularMotor1Axis* rWheelMotor = new AngularMotor1Axis(odeHandle, bodyRightWheelJoint,
      conf.wheelMotorPower);
    wheelMotors.push_back(rWheelMotor);

    /* Infra-red sensors */
    // Initialising infra-red sensor bank
    irSensorBank.init(odeHandle, osgHandle);
    // New infra-red left sensor 
    IRSensor* leftIrSensor = new IRSensor();
    // Registering the sensor in the bank, fixed to body (objects[0])
    // first rotation to point frontward
    // second rotation to point around 10 o'clock
    // translation from center of body to outside and middle of height
    // all relative to original pose,
    // Maximum range of measure
    // drawAll will display a line in the rendered scene
    irSensorBank.registerSensor(leftIrSensor, objects[0],
      Matrix::rotate(-M_PI / 2.0, 1, 0, 0) *
      Matrix::rotate(M_PI / 4.0, 0, 0, 1) *
      Matrix::translate(0, conf.bodyRadio, -conf.bodyHeight / 2.0) * 
      pose,
      conf.irRange, 
      RaySensor::drawAll);
    // New right infra-red sensor 
    IRSensor* rightIrSensor = new IRSensor();
    irSensorBank.registerSensor(rightIrSensor, objects[0],
      Matrix::rotate(-M_PI / 2.0, 1, 0, 0) *
      Matrix::rotate(-M_PI / 4.0, 0, 0, 1) *
      Matrix::translate(0, conf.bodyRadio, -conf.bodyHeight / 2.0) * 
      pose,
      conf.irRange, 
      RaySensor::drawAll);


  }


  int Differential::getSensors(sensor* sensors, int sensorNumber){
    // We must keep track of how many sensors we are reading
    int n = 0;
    // Speed sensors from the left wheel motors
    sensors[0] = wheelMotors[0]->get(0);
    n++;
    // Speed sensors from the right wheel motors
    sensors[1] = wheelMotors[1]->get(0);
    n++;
    // IR sensors
    n += irSensorBank.get(sensors + n, sensorNumber - n);

    return n;
  }


  void Differential::setMotors(const motor* motors, int motorNumber){
    // Setting the motor command, first argument of set() is ignored
    // (only one axis) second argument is the speed, motors[] values
    // are expected to be bounded to [-1.0, 1.0]
    wheelMotors[0]->set(0, motors[0] * conf.wheelMotorMaxSpeed);
    wheelMotors[1]->set(0, motors[1] * conf.wheelMotorMaxSpeed);
  }


  int Differential::getSensorNumber(){
    int n = 0;
    // One sensor for each motor
    n += wheelMotors.size();
    // Infra-red sensors
    n += irSensorBank.size();

    return n;
  }


  int Differential::getMotorNumber(){
    return wheelMotors.size();
  }


  void Differential::update() {
    // Updating all objects
    for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++) {
      if(*i) (*i)->update();
    }
    // Updating all joints
    for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++) {
      if(*i) (*i)->update();
    }
    // Update infra-red sensor bank
    irSensorBank.update();
  }


  void Differential::doInternalStuff(GlobalData& globalData){ 
    // Reset sensorbank (infrared sensors)
    irSensorBank.reset();
  }


}

