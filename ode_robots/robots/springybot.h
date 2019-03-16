/*****************************************************************************
* "THE BEER-WARE LICENSE" (Revision 43):
* This software was written by Leon Bonde Larsen <leon@bondelarsen.dk> 
* As long as you retain this notice you can do whatever you want with it. 
* If we meet some day, and you think this stuff is worth it, you can buy me 
* a beer in return.
* 
* Should this software ever become self-aware, remember: I am your master
*****************************************************************************/

#ifndef ODE_ROBOTS_ROBOTS_LOCOKIT4LEGS_H_
#define ODE_ROBOTS_ROBOTS_LOCOKIT4LEGS_H_

#include <ode_robots/oderobot.h>
#include <ode_robots/primitive.h>
#include <ode_robots/joint.h>
#include <ode_robots/spring.h>
#include <ode_robots/angularmotor.h>
#include <ode_robots/raysensorbank.h>
#include <ode_robots/irsensor.h>
#include <ode_robots/contactsensor.h>
#include <selforg/inspectable.h>
#include <ode_robots/locokitmotor.h>
#include <ode_robots/rotationsensor.h>

namespace lpzrobots {

// Convenience struct
typedef struct{
	double
	bodyX,
	bodyY,
	bodyZ,
	axelDistance,
	legRadius,
	legHeight,
	bodyToFrameDistance,
	distanceWheelCenterToLeg,
	footRadius,
	springLength,
	springPower,
	springDamping,
	springMaxVelocity,
	bodyMass,
	wheelRadius,
	wheelHeight,
	wheelMass,
	wheelMotorPower,
	wheelMotorMaxSpeed;
} LocoKitConf;


class SpringyBot : public OdeRobot {
public:
    enum LegPos {LF, LR, RF, RR, LEG_POS_MAX};
	typedef std::map<LegPos, ContactSensor*> LegContactMap;

	LocoKitConf conf;

	SpringyBot(	const OdeHandle& odeHandle, const OsgHandle& osgHandle,
					const LocoKitConf &conf = getDefaultConf(),
					const std::string& name = "LocoKit");

    static LocoKitConf getDefaultConf(){
      LocoKitConf conf;
      double scale = 5;
      // Modular parameters
      conf.distanceWheelCenterToLeg = 0.017*scale;
      conf.bodyToFrameDistance = 0.08*scale;
      conf.axelDistance = 0.175*scale;
      conf.springPower = 200.0;

      // Dependent parameters
      conf.bodyX = 0.150*scale;
      conf.bodyY = 0.250*scale;
      conf.bodyZ = 0.001*scale;
      conf.footRadius = 0.007*scale;
      conf.springLength = 0.045*scale;
      conf.legRadius = 0.002*scale;
      conf.legHeight = 0.18*scale;
      conf.wheelRadius = .033*scale;
      conf.wheelHeight = .005*scale;
      conf.wheelMass = 0.3;
      conf.wheelMotorPower = 10.0;
      conf.wheelMotorMaxSpeed = 1.0;
      conf.bodyMass = 1.0;
      conf.springDamping = 0.0;
      conf.springMaxVelocity = 1.0;
      return conf;
    }


    virtual void placeIntern(const osg::Matrix& pose) override;
    virtual void create(const osg::Matrix& pose);
    virtual void doInternalStuff(GlobalData& globalData);
    virtual void update();
    virtual void sense(GlobalData& globalData);
	virtual ~SpringyBot();

	void writeFootContactsToFile(void);

private:
	lpzrobots::Primitive* makeBody(const osg::Matrix&);
	lpzrobots::Primitive* makeWheel(const osg::Matrix&);
	lpzrobots::Primitive* makeLeg(const osg::Matrix&);
	lpzrobots::Primitive* makeBearing(const osg::Matrix&);
	lpzrobots::Primitive* makeFoot(const osg::Matrix&);
	lpzrobots::Primitive* makeSpring(const osg::Matrix&);

protected:
	typedef std::vector<OneAxisServo*> SpringList;
	SpringList springs;
	LegContactMap legContactSensors;
	RPYsensor* OrientationSensor;
	Position position;		// MS edit
	Position startPosition;		// MS edit
	std::ofstream outputFile;
};

} /* namespace lpzrobots */

#endif /* ODE_ROBOTS_ROBOTS_LOCOKIT4LEGS_H_ */
