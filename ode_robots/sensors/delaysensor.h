/***************************************************************************
 *   Copyright (C) 208-2015 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
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
#ifndef __DELAYSENSOR_H
#define __DELAYSENSOR_H

#include "sensor.h"
#include <selforg/ringbuffer.h>

namespace lpzrobots {

/// Class to wrap sensors and feed its delayed values.
class DelaySensor : public virtual Sensor, public Configurable {
public:
  DelaySensor(std::shared_ptr<Sensor> sensor)
  : childSensor(sensor), time(0) {
    addParameterDef("delay",&delay, 5, 0, 50, "delay in steps for sensor values");
    Configurable* c = dynamic_cast<Configurable*>(childSensor.get());
    if(c) {
      addConfigurable(c);
      setName("Delay-of-" + c->getName());
    }else setName("DelaySensor");

  }

  virtual ~DelaySensor(){
    for(int k=0; k<buffer.getBufferSize(); k++){
      if(buffer[k]) delete[] buffer[k];
    }
  }

  virtual void init(Primitive* own, Joint* joint = 0) override {
    assert(childSensor.get());
    number = childSensor->getSensorNumber();
    buffer.init(buffersize,0);
    for(int k=0; k<buffersize; k++){
      buffer[k]=new double[number];
      memset(buffer[k],0,sizeof(double)*number);
    }
    time = 0;
  };

  virtual bool sense(const GlobalData& globaldata) override {
    assert(childSensor);
    childSensor->sense(globaldata);
    time++;
    return true;
  };

  virtual int getSensorNumber() const override { return number;};

  virtual int get(sensor* sensors, int length) const override {
    assert(length>=number);
    childSensor->get(buffer[time],number);
    memcpy(sensors, buffer[time-delay],sizeof(double)*number);
    return number;
  }
  virtual std::list<sensor> getList() const override {
    return getListOfArray();
  };

  virtual std::list<SensorMotorInfo> getSensorInfos() const {
    std::list<SensorMotorInfo> l = childSensor->getSensorInfos();
    for (auto& i : l){
      i.name = i.name + "-delayed";
    }
    return l;
  };

protected:
  const int buffersize=250;
  int number;
  std::shared_ptr<Sensor> childSensor;
  RingBuffer<double*> buffer;
  int delay;
  long time;
};

}

#endif
