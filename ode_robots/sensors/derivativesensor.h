#ifndef __DERIVATIVESENSOR_H
#define __DERIVATIVESENSOR_H

#include "sensor.h"

namespace lpzrobots {

  /**
   * Class for measuring (time) derivatives of a given sensor.
   */
  class DerivativeSensor : public Sensor {
  public:

    /**
     * @param attachedSensor Sensor of which the derivative is computed
     * @param factor Scaling factor for derivative
     */
    DerivativeSensor(Sensor* attachedSensor, double factor = 1);

    virtual ~DerivativeSensor() {}

    virtual void init(Primitive* own, Joint* joint = 0);

    virtual int getSensorNumber() const;

    virtual bool sense(const GlobalData& globaldata);

    virtual std::list<sensor> getList() const;

  protected:
    //Current time step of the simulation
    double timeStepSize;
    //Values of last time step
    mutable std::list<sensor> oldValues;
    //Sensor of which to measure derivatives
    Sensor* attachedSensor;
    //Scaling factor for derivative (if derivative is too small)
    double factor;

  };


}

#endif
