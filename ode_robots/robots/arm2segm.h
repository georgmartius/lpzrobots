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
 *   Revision 1.3  2005-09-22 12:24:36  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.2  2005/09/20 11:17:29  fhesse
 *   smaller changes, needs clean up and comments
 *
 *   Revision 1.1  2005/07/28 10:22:55  fhesse
 *   initial version,
 *   known bugs: when calling show params an
 *   "pure virtual function called" error  happens
 *
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __ARM2SEGM_H
#define __ARM2SEGM_H

#include "abstractrobot.h"
#include "configurable.h"


typedef struct {
public:
  int armAnzahl; ///  number of snake elements
  double gliederLaenge; /// length of one snake element
  double gliederDurchmesser; ///  diameter of a snake element
  /**  distance between two snake elements; 
       0 means there is a distance of the length of one snake element 
       between each snake element an its successor */
  double gliederAbstand; 
  double gliederMasse; ///  mass of one snake element
  double maxMotorKraft; ///  maximal force used by the motors of the snake
  double factorForce; ///  factor for the speed, which the motors of the snake use
  double factorSensors; /// sensors values are multiplied with this value
} arm2SegmConf;



class Arm2Segm : public AbstractRobot, public Configurable{
public:
  
  Arm2Segm(const OdeHandle& odeHandle);

  virtual ~Arm2Segm(){};

  /**
   * draws the vehicle
   */
  virtual void draw();

  /** sets the vehicle to position pos, sets color to c, and creates robot if necessary
      @params pos desired position of the robot in struct Position
      @param c desired color for the robot in struct Color
  */
  virtual void place(Position pos , Color *c = 0);

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
  virtual int getSensorNumber(){
    return sensorno;
  };

  /** returns number of motors
   */
  virtual int getMotorNumber(){
    return motorno;
  };

  /** returns position of robot 
      @return position robot position in struct Position  
  */
  virtual Position getPosition();

  /** returns a vector with the positions of all segments of the robot
      @param poslist vector of positions (of all robot segments) 
      @return length of the list
  */
  virtual int getSegmentsPosition(vector<Position> &poslist);

  virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);
  /** this function is called in each timestep. It should perform robot-internal checks, 
      like space-internal collision detection, sensor resets/update etc.
      @param GlobalData structure that contains global data from the simulation environment
   */
  virtual void doInternalStuff(const GlobalData& globalData);

  /// returns the name of the object (with version number)
  virtual constparamkey getName() const {return name; } 
  
  /** The list of all parameters with there value as allocated lists.
      @param keylist,vallist will be allocated with malloc (free it after use!)
      @return length of the lists
  */
  virtual int getParamList(paramkey*& keylist,paramval*& vallist) const;
  
  virtual paramval getParam(paramkey key) const;
  
  virtual bool setParam(paramkey key, paramval val);


protected:

  /** creates vehicle at desired position 
      @param pos struct Position with desired position
  */
  virtual void create(Position pos); 

  /** destroys vehicle and space
   */
  virtual void destroy();
  static void mycallback(void *data, dGeomID o1, dGeomID o2);

  double dBodyGetPositionAll ( dBodyID basis , int para );



  static const int  armanzahl= 3;
  //Object km[armanzahl-1]; //Armglieder

  //  dJointID j[armanzahl+1];
    dJointID jm[armanzahl+1];

    double old_angle[armanzahl+1];
    Object object[armanzahl+1];  
    dJointID joint[armanzahl+1]; 

    char name[50];    
    paramval factorMotors;
    paramval factorSensors;
    paramval avgMotor;
    paramval maxMotorKraft;


  int segmentsno;    // number of motorsvehicle segments



  double gelenkabstand;
  double SOCKEL_LAENGE;
  double SOCKEL_BREITE;
  double SOCKEL_HOEHE; 
  double SOCKEL_MASSE;

  double ARMDICKE;
  double ARMLAENGE;
  double ARMABSTAND;
  double ARMMASSE;
  /*
  double length;  // chassis length
  double width;  // chassis width
  double height;   // chassis height
  double radius;  // wheel radius
  double wheelthickness; // thickness of the wheels  
  double cmass;    // chassis mass
  double wmass;    // wheel mass
  */
  int sensorno;      //number of sensors
  int motorno;       // number of motors

  /*  double speed;    // 

  Position initial_pos;    // initial position of robot
  double max_force;        // maximal force for motors
  */
  bool created;      // true if robot was created



  dSpaceID arm_space;
};

#endif
