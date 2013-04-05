/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   $Log$
 *   Revision 1.4  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.3  2009/12/04 18:51:59  fhesse
 *   invertnchannelcontroller has bias (changeable in constructor) now
 *   neuronworld has linear neuron now (changeable in conf)
 *
 *   Revision 1.2  2009/12/01 13:35:50  fhesse
 *   minor changes
 *
 *   Revision 1.1  2009/09/22 08:21:49  fhesse
 *   world is a schmitt trigger neuron
 *   only 1 DOF so far
 *
 *
 ***************************************************************************/

  /*
    "Robot" consisting of Schmitt Trigger Neurons (see Papers of Pasemann ets all, e.g. Huelse and Pasemann 2002)
    */

#ifndef __NEURONWORLD_H
#define __NEURONWORLD_H


#include "ode_robots/oderobot.h"
#include "ode_robots/primitive.h"

#include <selforg/matrix.h>

namespace lpzrobots {


enum NeuronType{linear, schmitt_trigger};

typedef struct {
public:
  double     theta_const;  ///<  constant part of bias
  double     gamma;        ///<  Dissipation
  double     w;            ///<  weight
  NeuronType neuron_type;  ///< type of neuron used as world
} NeuronWorldConf;


  /**
   *
   */
  class NeuronWorld : public OdeRobot{
  public:

    NeuronWorld(const OdeHandle& odeHandle, const OsgHandle& osgHandle, int sensornumber, int motornumber, const NeuronWorldConf& conf, const std::string& name="NeuronWorld");

  static NeuronWorldConf getDefaultConf(){
    NeuronWorldConf conf;
    conf.theta_const = 0;    //  constant part of bias
    conf.gamma = 0;          //  Dissipation
    conf.w = 0;         //  weight
    conf.neuron_type = schmitt_trigger;
    return conf;
  }


    virtual ~NeuronWorld();

    virtual void update() {}

    /** sets the pose of the vehicle
        @param pose desired 4x4 pose matrix
    */
    virtual void place(const osg::Matrix& pose) {}

    /** returns actual sensorvalues
        @param sensors sensors scaled to [-1,1]
        @param sensornumber length of the sensor array
        @return number of actually written sensors
    */
    virtual int getSensors(sensor* sensors, int sensornumber);

    /** sets actual motorcommands
        @param motors motors scaled to [-1,1]
        @param motornumber length of the motor array
    */
    virtual void setMotors(const motor* motors, int motornumber);

    /** returns number of sensors
     */
    virtual int getSensorNumber() {return sensorno; }

    /** returns number of motors
     */
    virtual int getMotorNumber() {return motorno; }

    /** this function is called in each timestep. It should perform robot-internal checks,
        like space-internal collision detection, sensor resets/update etc.
        @param globalData structure that contains global data from the simulation environment
    */
    virtual void doInternalStuff(GlobalData& globalData) {}


  protected:
    /** the main object of the robot, which is used for position and speed tracking */
    virtual Primitive* getMainPrimitive() const { return dummy; }

    /// neuron transfer function
    static double g(double z)
    {
      //return 1/(1+exp(-z));
      return tanh(z);
    };

  protected:
    DummyPrimitive *dummy;
    int sensorno;      //number of sensors
    int motorno;       // number of motors
    motor* motors;

    matrix::Matrix a; // Neuron Output Matrix
    matrix::Matrix gamma; // Dissipation
    matrix::Matrix theta; // Controller Bias (constant part + input)
    matrix::Matrix theta_const; // Constant part of Controller Bias
    matrix::Matrix W; // Wheight Matrix
    //matrix::Matrix* x_buffer;
    //matrix::Matrix* y_buffer;

    NeuronWorldConf conf;

  } ;

}

#endif

