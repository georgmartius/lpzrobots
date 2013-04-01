// Include libraries
//#include <ode-dbl/ode.h>
//#include <assert.h>
#include "differential.h"
//#include <osg/Matrix>


// Using namespaces
using namespace osg;
using namespace std;
namespace lpzrobots{

  Differential::Differential(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                const DifferentialConf& conf, const std::string& name)
    : OdeRobot(odeHandle, osgHandle, name, "revision 1.0"), conf(conf){
    }


  Differential::~Differential(){
    //destroy();
  }


  void Differential::place(const osg::Matrix& pose){
    // Movig robot upward so wheel are not stuck on the ground
    Matrix intialPose;
    intialPose = pose * Matrix::translate(Vec3(0, 0, conf.wheelRadio));
    
    // Creating body
    body = new Cylinder(conf.bodyRadio, conf.bodyHeight);
    body->init(odeHandle, conf.bodyMass, osgHandle);
    body->setPose(intialPose);
  }


  int Differential::getSensors(sensor* sensors, int sensorNumber){
    return sensorNumber;
  }


  void Differential::setMotors(const motor* motors, int motorNumber){
    //return motorNumber;
  }


  int Differential::getSensorNumber(){
    return 1; //sensors.size();
  }


  int Differential::getMotorNumber(){
    return 1;
  }

  void Differential::update() {
  }

    
}

