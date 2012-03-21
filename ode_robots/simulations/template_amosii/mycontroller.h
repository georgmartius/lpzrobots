/***************************************************************************
 *   Copyright (C) 2010 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   ANY COMMERCIAL USE FORBIDDEN!                                         *
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
 *                                            * 
 *                                                                         *
 *   $Log: tripodgate12dof.h,v $
 *                                                                         *
 ***************************************************************************/
#ifndef __MYCONTROLLER_H
#define __MYCONTROLLER_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <selforg/configurable.h>

#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>

#include <selforg/matrix.h>
// forward declarations
class adaptiveSO2CPGHebb;

struct MyControllerConf {

};


class MyController : public AbstractController {

public:
    MyController(const MyControllerConf& conf = getDefaultConf());
    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

    virtual ~MyController();

    // returns the name of the object (with version number)
    virtual paramkey getName() const;

    // returns the number of sensors the controller was initialised with or 0
    // if not initialised
    virtual int getSensorNumber() const;

    // returns the mumber of motors the controller was initialised with or 0 if
    // not initialised
    virtual int getMotorNumber() const;

    // performs one step (includes learning).
    // Calulates motor commands from sensor inputs.
    virtual void step(const sensor* , int number_sensors, motor* , int number_motors);

    /// performs one step without learning. Calulates motor commands from sensor inputs.
    virtual void stepNoLearning(const sensor* , int number_sensors,
                  motor* , int number_motors);


    /***** STOREABLE ****/
    // stores the controller values to a given file.
    virtual bool store(FILE* f) const;
    // loads the controller values from a given file.
    virtual bool restore(FILE* f);

    static MyControllerConf getDefaultConf();

protected:
    unsigned short number_sen;
    unsigned short number_mot;

    int t;
    paramkey name;
    MyControllerConf conf;
private:
};

#endif


