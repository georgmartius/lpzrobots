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

#include <assert.h>
#include <osg/Matrix>

#include <ode_robots/twowheeled.h>
#include <ode_robots/camerasensors.h>

using namespace std;

namespace lpzrobots {

  TwoWheeled::TwoWheeled(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                         TwoWheeledConf conf, const std::string& name)
    : Nimm2(odeHandle, osgHandle, conf.n2cfg, name), conf(conf)
  {
    if(conf.useCamera){
      cam = new Camera(this->conf.camcfg);
      if(!conf.camSensor)
        conf.camSensor = new DirectCameraSensor();

      conf.camSensor->setInitData( cam, this->odeHandle,
                                   this->osgHandle.changeColor(Color(0.2,0.2,0.2)),
                                   conf.camPos);

      this->conf.sensors.push_back(conf.camSensor);
    }else{ // delete the processors
      FOREACH(ImageProcessors, conf.camcfg.processors, ip){
        if(*ip) delete *ip;
      }
      conf.camcfg.processors.clear();
    }
  };


  TwoWheeled::~TwoWheeled(){
    destroy();
    FOREACH(list<Sensor*>, conf.sensors, i){
      if(*i) delete *i;
    }
    conf.sensors.clear();
  }

  int TwoWheeled::getSensorNumberIntern(){
    int s=0;
    FOREACHC(list<Sensor*>, conf.sensors, i){
      s += (*i)->getSensorNumber();
    }
    return Nimm2::getSensorNumberIntern() + s;
  }

  int TwoWheeled::getSensorsIntern(double* sensors, int sensornumber){
    int len = Nimm2::getSensorsIntern(sensors,sensornumber);
    FOREACH(list<Sensor*>, conf.sensors, i){
      len += (*i)->get(sensors + len, sensornumber - len);
    }
    return len;
  };

  void TwoWheeled::update(){
    Nimm2::update();
    FOREACHC(list<Sensor*>, conf.sensors, i){
      (*i)->update();
    }
  }

  void TwoWheeled::sense(GlobalData& globalData){
    Nimm2::sense(globalData);
    FOREACH(list<Sensor*>, conf.sensors, i){
      (*i)->sense(globalData);
    }
  }

  /** creates vehicle at desired position
      @param pos struct Position with desired position
  */
  void TwoWheeled::create(const osg::Matrix& pose){
    Nimm2::create(pose);
    Primitive* p = getMainPrimitive();
    FOREACH(list<Sensor*>, conf.sensors, i){
      (*i)->init(p);
    }
  };


  /** destroys vehicle and space
   */
  void TwoWheeled::destroy(){
    Nimm2::destroy();
  }

}
