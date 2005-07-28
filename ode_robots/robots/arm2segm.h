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
 *   Revision 1.1  2005-07-28 10:22:55  fhesse
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

class Arm2Segm : public AbstractRobot, public Configurable{
public:
  
  Arm2Segm(dWorldID w, dSpaceID s, dJointGroupID c);

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



  static const int  armanzahl= 2;
  //Object km[armanzahl-1]; //Armglieder

  //  dJointID j[armanzahl+1];
    dJointID jm[armanzahl+1];

    double old_angle[3];
    Object object[3];  
    dJointID joint[3]; 

    char name[50];    
    paramval factorMotors;
    paramval factorSensors;
    paramval avgMotor;
    paramval maxMotorKraft;


  int segmentsno;    // number of motorsvehicle segments



  double BASIS_START_X;
  double BASIS_START_Y;
  double BASIS_BODENABSTAND;
  double GELENKABSTAND;
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
