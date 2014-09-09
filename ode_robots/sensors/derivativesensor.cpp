#include "derivativesensor.h"

namespace lpzrobots {

  DerivativeSensor::DerivativeSensor(Sensor* attachedSensor, double factor){
    this->attachedSensor = attachedSensor;
    this->factor = factor;
    this->timeStepSize = 0;
  }

  void DerivativeSensor::init(Primitive* own, Joint* joint){
    this->attachedSensor->init(own, joint);
    this->oldValues.resize(this->attachedSensor->getSensorNumber());
  }

  int DerivativeSensor::getSensorNumber() const{
    return this->attachedSensor->getSensorNumber();
  }

  bool DerivativeSensor::sense(const GlobalData& globaldata){
    this->timeStepSize = globaldata.odeConfig.simStepSize;
    this->attachedSensor->sense(globaldata);
    return true;
  }

  std::list<sensor> DerivativeSensor::getList() const {
    std::list<sensor> newValues = this->attachedSensor->getList();
    std::list<sensor> derivatives(newValues.size());
    auto iterNewValues = newValues.cbegin();
    auto iterOldValues = this->oldValues.begin();
    for(sensor& d: derivatives){
      d = (*iterNewValues - *iterOldValues) / this->timeStepSize;
      d *= this->factor;
      *iterOldValues = *iterNewValues;
      iterOldValues++;
      iterNewValues++;
    }

    return derivatives;
  }

}

