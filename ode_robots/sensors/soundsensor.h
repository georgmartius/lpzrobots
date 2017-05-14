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
#ifndef           SOUNDSENSOR_H_
# define           SOUNDSENSOR_H_

#include "sensor.h"

namespace lpzrobots {

  /** Sound sensor
      with possible direction and frequency detection and also
      distance dependence (Not implemented yet)
      This works, but is not very well tested and documented.
      The angle detection also works without sound and so on.
      Take a look at the code before using it.
  */
  class SoundSensor: public Sensor {
  public:
    /** Segments: for each segement and level we get the
        Angle: for each level we get a triple (intensity, sin(angle), cos(angle))
               multiple sound sources are averaged (weighted by intensity)
        AngleVel: for each level we get a tuple (intensity, angle-velocity)
     */
    enum Measure { Segments, Angle, AngleVel };
    // TODO: add simple Microphone and probably some timing

    /** @param dim Up-axis of the robot (sometimes it is not Z)
        @param measure what to measure @see Measure
    */
    SoundSensor(Dimensions dim = Z, Measure measure = Angle,
                int segments=1, int levels=1, double maxDistance=50, double noisestrength=0.1);
    virtual ~SoundSensor();

    virtual void init(Primitive* own, Joint* joint = 0) override { this->own = own;}

    virtual bool sense(const GlobalData& globaldata)  override;

    /// default implementation is a linear decrease in intensity until it is 0 at maxDistance
    virtual float distanceDependency(const Sound& s, double distance);

    virtual int getSensorNumber() const  override;

    virtual std::list<sensor> getList() const override;

    virtual int get(sensor* sensors, int length) const override;

  private:
    short dim; ///< the axis in which the sensor is selective around
    Measure measure; ///< how to measure
    int segments;
    int levels;
    double maxDistance;
    double noisestrength;

    double* val;
    double* oldangle;

    Primitive* own;

  };




}

#endif             /* !SOUNDSENSOR_H_ */
