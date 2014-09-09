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
#ifndef __SENSORMOTORINFOABLE_H
#define __SENSORMOTORINFOABLE_H

#include <list>
#include <vector>
#include <functional>

#include <selforg/sensormotorinfo.h>

namespace lpzrobots {

  /** Abstract class for giving names to sensors and motors
  */
  class SensorMotorInfoAble {
  public:
    /// function that returns the name given the index
    typedef std::function<std::string(int)> NamingFunction;

    SensorMotorInfoAble() : func(defaultNameing), baseinfo("Unknown") {}

    // sets the base name that is used to construct the names for each item with the nameing function
    void setBaseName(const std::string& basename) {
      this->baseinfo.name=basename;
    }

    // sets the base information for sensor or motor (the name is considered as base name)
    void setBaseInfo(const SensorMotorInfo& baseinfo) {
      this->baseinfo=baseinfo;
    }
    // sets the base information for sensor or motor (the name is considered as base name)
    SensorMotorInfo getBaseInfo() {
      return this->baseinfo;
    }

    void setNamingFunc(const NamingFunction& func){
      this->func=func;
    }

    NamingFunction  getNamingFunc() const {
      return this->func;
    }

    /// set names explicitly (basename is anyway suffixed)
    void setNames(const std::vector<std::string>& names){
      this->func=[names](int index) {
        if (index>=(int)names.size()) return names.back() + "Unknown";
        else return names[index];
      };
    }

    /// returns the name of a single item. Typically called from within Sensor and Motor class.
    std::string getName(int index) const {
      return baseinfo.name + func(index);
    }

    /** get all infos. Typically called from within Sensor and Motor class.
     */
    std::list<SensorMotorInfo> getInfos(int number) const {
      std::list<SensorMotorInfo> l;
      for(int i=0; i<number; i++){
        SensorMotorInfo info = baseinfo;
        info.name = getName(i);
        info.index = i;
        l.push_back(info);
      }
      return l;
    }



    /// the default implementation is for index==0: basename, otherwise basename + (index+1)
    static std::string defaultNameing(int index) {
      if(index==0)
        return "";
      else
        return std::to_string(index+1);
    }

  protected:
    NamingFunction func;
    SensorMotorInfo baseinfo;
  };
}

#endif
