/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *    Guillaume de Chambrier <s0672742 at sms dot ed dot ac dot uk>        *
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


#ifndef __HEXAPOD_H
#define __HEXAPOD_H

#include <selforg/inspectable.h>

#include <ode_robots/oderobot.h>
#include <ode_robots/raysensorbank.h>
#include <ode_robots/contactsensor.h>

namespace lpzrobots {

  class Primitive;
  class Joint;
  class OneAxisServo;
  class TwoAxisServo;


  struct HexapodConf {
    double size;       ///< scaling factor for robot (diameter of body)
    double legLength;  ///< length of the legs in units of size
    int    legNumber;  ///<  number of snake elements
    double width;      ///< body with in units of size
    double height;     ///< body with in units of size
    double mass;       ///< chassis mass
    double percentageBodyMass; ///< relation between bodymass(trunk) and rest

    double coxaPower; ///< maximal force for at hip joint motors
    double coxaJointLimitV; ///< angle range for vertical direction of legs
    double coxaJointLimitH; ///< angle range for horizontal direction of legs
    double coxaDamping;     ///< damping of hip joint servos
    double coxaSpeed;       ///< speed of the hip servo

    bool useTebiaJoints;    ///< whether to use joints at the knees
    bool useTebiaMotors;    ///< if true tebia joints are actuated, otherwise springs
    double tebiaPower;       ///< spring strength in the knees
    double tebiaJointLimit;  ///< angle range for knees
    double tebiaOffset;      ///< angle offset for knees
    double tebiaDamping; ///< damping in the knees

    double legSpreading;   ///< angle at which legs are spread to the front and back
    double backLegFactor;   ///< size factor for back legs

    bool tarsus;                  ///< whether to use a tarsus or not
    int numTarsusSections; ///< the number of sections in the tarsus
    bool useTarsusJoints; ///< whether to use joints on the tarsus
    bool useBigBox;       ///< whether to use a big invisible box on the back of the robot

    bool useWhiskers;         ///< if true add whiskers to robot
    bool useActiveWhisker;    ///< if true whiskers have motorized joints, otherwise springs
    double whiskerSpread;   ///< angle by which the wiskers are spread

    double T; ///< T is the for the time for calculating the cost of transport over time
    double *v;

    bool ignoreInternalCollisions;

    bool calculateEnergy;

    bool useContactSensors;
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
  };

  struct Leg {
    int legID;
    dGeomID geomid;
    dBodyID bodyID;

    dJointID joint;
  };


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

    virtual ~Hexapod(){ destroy(); };

    static HexapodConf getDefaultConf(){
      HexapodConf c;
      c.size               = 1;
      c.width              = 1.0/3.0; //1.0/1.5
      c.height             = 1.0/8.0; //1.0/4.0
      c.legNumber          = 6;
      c.legLength          = 0.6;
      c.percentageBodyMass = 0.7;
      c.mass               = 1.0;
      c.v                  = new double[1];
      c.coxaPower          = 1;
      c.coxaJointLimitV    = M_PI/8.0; ///< angle range for vertical direction of legs
      c.coxaJointLimitH    = M_PI/4.0;
      c.coxaDamping        = 0.0; // Georg: no damping required for new servos
      c.coxaSpeed          = 30; // The speed calculates how it works
      c.T                  = 1.0;

      c.legSpreading  = M_PI/12.0;
      c.backLegFactor = 1.0;

      c.useTebiaJoints  = true;
      c.useTebiaMotors  = false;
      c.tebiaPower      = 1.2;
      c.tebiaJointLimit = M_PI/4; // +- 45 degree
      c.tebiaDamping    = 0.01; // Georg: the damping reduces the swinging of the system
      c.tebiaOffset     = 0.0;

      c.tarsus            = true;
      c.numTarsusSections = 2;
      c.useTarsusJoints   = true;
      c.useBigBox         = true;

      c.whiskerSpread    = M_PI/10.0;
      c.useWhiskers      = true;
      c.useActiveWhisker = false;

      c.ignoreInternalCollisions = true;

      c.calculateEnergy = false;

      c.useContactSensors  = false;
      c.legContacts        = new int[6];
      c.irSensors          = false;
      c.irFront            = false;
      c.irBack             = false;
      c.irLeft             = false;
      c.irRight            = false;
      c.irRangeFront       = 3;
      c.irRangeBack        = 2;
      c.irRangeLeft        = 2;
      c.irRangeRight       = 2;
      //      c.elasticity = 10;
      return c;
    }


    /** sets the pose of the vehicle
        @param pose desired pose matrix
    */
    virtual void placeIntern(const osg::Matrix& pose);


    /** this function is called in each timestep. It should perform robot-internal checks,
        like space-internal collision detection, sensor resets/update etc.
        @param globalData structure that contains global data from the simulation environment
    */
    virtual void doInternalStuff(GlobalData& globalData);


    /**
     * calculates the total energy consumption of all servos.
     */
    double round(double,int);

    virtual double energyConsumption();

    virtual double energyConsumpThroughtHeatLoss(const dReal *torques);

    virtual double outwardMechanicalPower(const dReal *torques,const dReal *angularV);

    virtual double costOfTransport(double E, double W, double V, double T);

    virtual double getMassOfRobot();

    virtual double *getPosition(){
            return position;
    }

    /******** CONFIGURABLE ***********/
    virtual void notifyOnChange(const paramkey& key);

    virtual void resetMotorPower(double power);

    virtual double getPower();

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

public:
    HexapodConf conf;
    double legmass;    // leg mass
    bool created;      // true if robot was created

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
    double timeCounter;
    double *position;
    std::vector<dReal*> pos_record;
    dMass *massOfobject;
    bool getPos1;
    double speed;

    std::vector<Leg> legContact;
    Leg* legContactArray;
    std::vector<dGeomID> footIDs;
  protected:
    // some objects explicitly needed for ignored collision pairs
    Primitive *trunk, *irbox, *stabaliserTransform, *bigboxtransform, *headtrans;
    std::vector<Primitive*> legs;
    std::vector<Primitive*> thorax;
    std::vector<Pos> thoraxPos;

    std::vector <std::shared_ptr<TwoAxisServo> > hipservos;
    std::vector <std::shared_ptr<OneAxisServo> > tebiaservos;
    std::vector <std::shared_ptr<OneAxisServo> > tarsussprings;
  };

}

#endif
