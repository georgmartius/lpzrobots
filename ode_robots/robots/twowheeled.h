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
#ifndef __TWO_WHEELED__
#define __TWO_WHEELED__

#include <ode_robots/nimm2.h>
#include <ode_robots/camera.h>
#include <ode_robots/camerasensor.h>
#include <ode_robots/imageprocessors.h>
#include <ode_robots/sensor.h>

namespace lpzrobots {

  typedef struct {
    Nimm2Conf n2cfg;   ///< configuration for underlying nimm2 robot
    CameraConf camcfg; ///< camera config. Allows to change the image processing
    bool useCamera;    ///< whether to use the camera
    osg::Matrix camPos; ///< relative pose of the camera
    /** camera sensor (converts image to sensor data)
        (if NULL then DirectCameraSensor() is used) */
    CameraSensor* camSensor;
    /// list of sensors that are mounted at the robot. (e.g.\ AxisOrientationSensor)
    std::list<Sensor*> sensors;
    /// adds a sensor to the list of sensors
    void addSensor(Sensor* s) { sensors.push_back(s); }
  } TwoWheeledConf;

  /** Robot is based on nimm2 with
      a camera installed
  */
  class TwoWheeled : public Nimm2{
  public:

    /**
     * constructor of twowheeled robot
     * @param odeHandle data structure for accessing ODE
     * @param osgHandle ata structure for accessing OSG
     * @param conf configuration structure
     * @param name name of the robot
     */
    TwoWheeled(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
               TwoWheeledConf conf, const std::string& name);

    static TwoWheeledConf getDefaultConf(){
      TwoWheeledConf conf;
      conf.n2cfg = Nimm2::getDefaultConf();
      conf.camcfg = Camera::getDefaultConf();
      conf.camcfg.width = 256;
      conf.camcfg.height = 64;
      conf.camcfg.fov    =  90;
      conf.camcfg.camSize = 0.08;
      conf.camcfg.processors.push_back(new HSVImgProc(false,1));
      // filter only Yellow color
      conf.camcfg.processors.push_back(new ColorFilterImgProc(true, .5,
                             HSVImgProc::Red+20, HSVImgProc::Green-20,100));
      // only two sensors for left and right visual field
      conf.camcfg.processors.push_back(new LineImgProc(true,20, 2));
      conf.useCamera = true;
      conf.camPos    = osg::Matrix::rotate(M_PI/2,0,0,1)
        * osg::Matrix::translate(-0.20,0,0.40);
      conf.camSensor = 0;
      return conf;
    }

    virtual ~TwoWheeled();

    virtual void update();

    virtual int getSensorNumberIntern();

    virtual int getSensorsIntern(double* sensors, int sensornumber);

    virtual void sense(GlobalData& globalData);


  protected:
    /** creates vehicle at desired pose
        @param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose);

    /** destroys vehicle and space
     */
    virtual void destroy();

    TwoWheeledConf conf;
    CameraSensor* camsensor;
    Camera* cam;
  };

}

#endif
