#include "rangeFinder.h"

namespace lpzrobots{

  void RangeFinder::init(Primitive* own, Joint* joint){
    this->RaySensorBank::init(own, joint);
    this->own = own;
  }


  void RangeFinder::registerSensorRange(int numBeams, double startAngle, double endAngle,
      double maxRange, double height,  RaySensor::rayDrawMode drawMode){

    double sensorAngle = startAngle;
    double angleStep = (endAngle - startAngle) / (numBeams - 1);
    //Register IR sensors in given angular range
    for(int i = 0; i < numBeams; i++){
      this->registerSensor(new RaySensor(),this->own, osg::Matrix::rotate(M_PI / 2, 0, 1, 0) *
          osg::Matrix::rotate(sensorAngle, 0, 0, 1)*
          osg::Matrix::translate(0,0, height), maxRange, drawMode);
      sensorAngle += angleStep;
    }
  }

}
