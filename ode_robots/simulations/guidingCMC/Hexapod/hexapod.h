/***************************************************************************
 *   Copyright (C) 2010 by                                                 *
 *    Guillaume de Chambrier <s0672742@sms.ed.ac.uk>                       *
 *    martius@informatik.uni-leipzig.de                                    *
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
 **************************************************************************/


#ifndef __HEXAPOD_H
#define __HEXAPOD_H

#include <selforg/inspectable.h>

#include <ode_robots/oderobot.h>
#include <ode_robots/raysensorbank.h>

namespace lpzrobots {

  class Primitive;
  class Joint;
  class OneAxisServo;
  class TwoAxisServo;


  typedef struct {
  public:
    double size;       ///< scaling factor for robot (diameter of body)
    double legLength;  ///< length of the legs in units of size
    int    legNumber;  ///<  number of snake elements
    double width;      ///< body with in units of size
    double height;     ///< body with in units of size
    double mass;       ///< chassis mass
    double relLegmass; ///< relative overall leg mass
    double percentageBodyMass;

    double coxaPower; ///< maximal force for at hip joint motors
    double coxaJointLimitV; ///< angle range for vertical direction of legs
    double coxaJointLimitH; ///< angle range for horizontal direction of legs
    double coxaDamping;     ///< damping of hip joint servos
    double coxaSpeed;       ///< speed of the hip servo

    bool useTebiaJoints;    /// whether to use joints at the knees
    double tebiaPower;       ///< spring strength in the knees
    double tebiaJointLimit;  ///< angle range for knees
    double tebiaDamping; ///< damping in the knees

    double T; ///< T is the for the time for calculating the cost of transport over time
    double *v;

    matrix::Matrix m;
    int *legContacts;
    double irSensors;
    bool irFront;
    bool irBack;
    bool irLeft;
    bool irRight;
    double irRangeFront;
    double irRangeBack;
    double irRangeLeft;
    double irRangeRight;
  } HexapodConf;

  typedef struct{
  public:
          int legID;
          dGeomID geomid;
          dBodyID bodyID;

          dJointID joint;
  } Leg;


  class Hexapod : public OdeRobot, public Inspectable {
  public:

    /**
     * constructor of VierBeiner robot
     * @param odeHandle data structure for accessing ODE
     * @param osgHandle ata structure for accessing OSG
     * @param conf configuration object
     */
    Hexapod(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const HexapodConf& conf,
               const std::string& name);

    virtual ~Hexapod(){};

    static HexapodConf getDefaultConf(){
      HexapodConf c;
      c.size       = 1;
      c.width      = 1.0/3.0; //1.0/1.5
      c.height     = 1.0/5.0; //1.0/4.0
      c.legNumber  = 6;
      c.legLength  = 0.6;
      c.percentageBodyMass = 0.7;
      c.mass       = 1.0;
      c.v = new double[1];
      c.relLegmass = 1;
      c.coxaPower  = 1;
      c.coxaJointLimitV = M_PI/8;  ///< angle range for vertical direction of legs
      c.coxaJointLimitH = M_PI/4;
      c.coxaDamping = 0.05; // body feeling (velocity servo)
      c.coxaSpeed   = 30; // The speed calculates how it works
      c.T = 1.0;

      c.useTebiaJoints = true;
      c.tebiaPower = 2;
      c.tebiaJointLimit = M_PI/4; // +- 45 degree
      c.tebiaDamping = 0.01;

      c.legContacts = new int[6];
      c.irSensors=false;
      c.irFront=false;
      c.irBack=false;
      c.irLeft=false;
      c.irRight=false;
      c.irRangeFront=3;
      c.irRangeBack=2;
      c.irRangeLeft=2;
      c.irRangeRight=2;
      //      c.elasticity = 10;
      return c;
    }

    /**
     * updates the OSG nodes of the vehicle
     */
    virtual void update();


    /** sets the pose of the vehicle
        @param pose desired pose matrix
    */
    virtual void place(const osg::Matrix& pose);

    /** returns actual sensorvalues
        @param sensors sensors scaled to [-1,1]
        @param sensornumbHexapod::getDefaultConf()er length of the sensor array
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
    virtual int getSensorNumber();

    /** returns number of motors
     */
    virtual int getMotorNumber();
    /** checks for internal collisions and treats them.
     *  In case of a treatment return true (collision will be ignored by other objects
     *  and the default routine)  else false (collision is passed to other objects and
     *  (if not treated) to the default routine).
     */
    virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);

    /** this function is called in each timestep. It should perform robot-internal checks,
        like space-internal collision detection, sensor resets/update etc.
        @param globalData structure that contains global data from the simulation environment
    */
    virtual void doInternalStuff(GlobalData& globalData);


   // virtual void Hexapod::updateLegTouch(int);

    /**
     * calculates the total energy consumption of all servos.
     */
    double round(double,int);

    virtual double energyConsumption();

    virtual double energyConsumpThroughtHeatLoss(const dReal *torques);

    virtual double outwardMechanicalPower(const dReal *torques,const dReal *angularV);

    virtual double costOfTransport(double E, double W, double V, double T);

    virtual double getMassOfRobot();

    /******** CONFIGURABLE ***********/
    virtual void notifyOnChange(const paramkey& key);

    /** the main object of the robot, which is used for position and speed tracking */
    virtual Primitive* getMainPrimitive() const { return objects[0]; }
  protected:

    /** creates vehicle at desired pose
        @param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose);

    /** destroys vehicle and space
     */
    virtual void destroy();


    HexapodConf conf;
    double legmass;    // leg mass
    int        countt;
    bool created;      // true if robot was created
    RaySensorBank irSensorBank; // a collection of ir sensors

  public:
    double costOfTran;
    double* energyOneStep; ///< energy consumption for one time step
    double E_t;        ///< energy consumption over a period t;
    bool recordGait;
    double *heights;
    double *angles;
  private:
    double hcorrection;
    bool *dones;
    bool check;
    double t;
    FILE* f;
    double timeCounter;
    double *pos1d;
    const dReal *pos1;
    const dReal *pos2;
    dMass *massOfobject;
    bool getPos1;
    double distance;
    double time;

    std::vector<Leg> legContact;
    Leg* legContactArray;
    std::vector<dGeomID> footIDs;
  protected:
    // some objects explicitly needed for ignored collision pairs
    Primitive *trunk, *irbox, *bigboxtransform, *headtrans;
    std::vector<Primitive*> legs;
    std::vector<Primitive*> thorax;
    std::vector<Pos> thoraxPos;
    std::vector<Primitive*> objects;  // all the objects
    std::vector<Joint*> joints; // joints legs
    std::vector <TwoAxisServo*> hipservos; // motor
    std::vector <OneAxisServo*> tebiasprings;
    std::vector <OneAxisServo*> whiskersprings;

  };

}

#endif
