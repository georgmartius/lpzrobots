/***************************************************************************
 *   Copyright (C) 2010 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 *   $Log: tripodgate12dof.cpp,v $                                         *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include <selforg/controller_misc.h>
#include "mycontroller.h"
#include "selforg/amosiisensormotordefinition.h"
using namespace matrix;
using namespace std;

MyController::MyController( const MyControllerConf& _conf)
    : AbstractController("MyController", "$Id: MyController.cpp,v 0.1 $"), conf(_conf)
{
  t=0;  
  // prepare name;
  Configurable::insertCVSInfo(name, "$RCSfile: MyController.cpp,v $",
          "$Revision: 0.1 $");
};

MyController::~MyController()
{
}

MyControllerConf MyController::getDefaultConf(){
    MyControllerConf c;
    return c;
}

MyController::paramkey MyController::getName() const
{
    return name;
}

int MyController::getMotorNumber() const  {
    return number_mot;
}


int MyController::getSensorNumber() const
{
    return number_sen;
}

void MyController::init(int sensornumber, int motornumber, RandGen* randGen)
{

}


/// performs one step (includes learning). Calulates motor commands from sensor inputs.
void MyController::step(const sensor* x_, int number_sensors,
        motor* y_, int number_motors)
{
    stepNoLearning(x_, number_sensors, y_, number_motors);
};

/// performs one step without learning. Calulates motor commands from sensor inputs.
void MyController::stepNoLearning(const sensor* x_, int number_sensors,
        motor* y, int number_motors){
    for (int i=0; i<number_motors; i++) y[i] = 0;
    const double omega = 0.1;
    const double A =  0.3;
    const double B = 0.3;
    const double dphi = M_PI;
    y[TL0_m] =  A*sin(omega*t + 0 * dphi);
    y[CL0_m] =  B*cos(omega*t + 0 * dphi);
    y[TL1_m] =  A*sin(omega*t + 1 * dphi);
    y[CL1_m] =  B*cos(omega*t + 1 * dphi);
    y[TL2_m] =  A*sin(omega*t + 2 * dphi);
    y[CL2_m] =  B*cos(omega*t + 2 * dphi);
    y[TR0_m] =  A*sin(omega*t + 3 * dphi);
    y[CR0_m] =  B*cos(omega*t + 3 * dphi);
    y[TR1_m] =  A*sin(omega*t + 4 * dphi);
    y[CR1_m] =  B*cos(omega*t + 4 * dphi);
    y[TR2_m] =  A*sin(omega*t + 5 * dphi);
    y[CR2_m] =  B*cos(omega*t + 5 * dphi);

    //y[FL1_m] = 2*sin(0.1*t);
    //y[FL2_m] = 2*sin(0.1*t);
    //y[FR0_m] = 2*sin(0.1*t);
    //y[FR1_m] = 2*sin(0.1*t);
    //y[FR2_m] = 2*sin(0.1*t);
    t++;
};
  
/** stores the controller values to a given file. */
bool MyController::store(FILE* f) const{
    return true;
}


/** loads the controller values from a given file. */
bool MyController::restore(FILE* f){
    return true;
}



  
