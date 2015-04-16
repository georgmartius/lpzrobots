/*
 * LocoKit4Legs.cpp
 *
 *  Created on: Dec 27, 2014
 *      Author: leon
 */

#include "springybot.h"

using namespace osg;
using namespace std;
namespace lpzrobots {

SpringyBot::SpringyBot(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const LocoKitConf& conf, const string& name)
: OdeRobot(odeHandle, osgHandle, name, "2.0"), conf(conf){
}

SpringyBot::~SpringyBot() {

}

void SpringyBot::placeIntern(const Matrix& pose){
	Matrix initialPose;
	initialPose = Matrix::translate(Vec3(0, 0, conf.footRadius+conf.legHeight+conf.springLength+conf.wheelRadius) * pose);
	create(initialPose);
	outputFile.open("footcontacts.csv", std::ios::app);
}

void SpringyBot::create(const Matrix& pose) {

	// Create and join body and frame
	auto body = makeBody(pose);
	auto frame = makeBody(Matrix::translate( 0.0, 0.0, -conf.bodyToFrameDistance) * pose );
	auto BodyFrameJoint = new FixedJoint(body, frame, body->getPosition());
	BodyFrameJoint->init(odeHandle, osgHandle, false);
	joints.push_back(BodyFrameJoint);
	OrientationSensor = new RPYsensor(RPYsensor::Axis,Sensor::X |Sensor::Y | Sensor::Z);
	OrientationSensor->init(body);

	// Create and join left front wheel
	auto lfRotation = Matrix::rotate(M_PI / 2.0, 0, 1, 0);
	auto lfTranslation = Matrix::translate( (conf.bodyX/2) + conf.wheelHeight / 2.0, conf.axelDistance/2, .0);
	Matrix lfWheelPose = lfRotation * lfTranslation * pose;
	auto lfWheel = makeWheel(lfWheelPose);
	auto bodyLeftFrontWheelJoint = new HingeJoint(body, lfWheel, lfWheel->getPosition(), Axis(0, 0, 1) * lfWheelPose);
	bodyLeftFrontWheelJoint->init(odeHandle, osgHandle, false);
	joints.push_back(bodyLeftFrontWheelJoint);
	auto lfMotor = std::make_shared<VelocityControlledLocoKitMotor>(odeHandle, bodyLeftFrontWheelJoint, conf.wheelMotorPower);
	lfMotor->setBaseName("left front motor");
	lfMotor->setVelovityFactor(conf.wheelMotorMaxSpeed);
	addSensor(lfMotor);
	addMotor(lfMotor);

	// Create and join left rear wheel
	auto lrRotation = Matrix::rotate(M_PI / 2.0, 0, 1, 0);
	auto lrTranslation = Matrix::translate( (conf.bodyX/2) + conf.wheelHeight / 2.0, -conf.axelDistance/2, .0);
	Matrix lrWheelPose = lrRotation * lrTranslation * pose;
	auto lrWheel = makeWheel(lrWheelPose);
	auto bodyLeftRearWheelJoint = new HingeJoint(body, lrWheel, lrWheel->getPosition(), Axis(0, 0, 1) * lrWheelPose);
	bodyLeftRearWheelJoint->init(odeHandle, osgHandle, false);
	joints.push_back(bodyLeftRearWheelJoint);
	auto lrMotor = std::make_shared<VelocityControlledLocoKitMotor>(odeHandle, bodyLeftRearWheelJoint, conf.wheelMotorPower);
	lrMotor->setBaseName("left rear motor");
	lrMotor->setVelovityFactor(conf.wheelMotorMaxSpeed);
	addSensor(lrMotor);
	addMotor(lrMotor);

	// Create and join right front wheel
	auto rfRotation = Matrix::rotate(M_PI / 2.0, 0, 1, 0);
	auto rfTranslation = Matrix::translate(-((conf.bodyX/2) + conf.wheelHeight / 2.0), conf.axelDistance/2, .0);
	Matrix rfWheelPose = rfRotation * rfTranslation * pose;
	auto rfWheel = makeWheel(rfWheelPose);
	auto bodyRightFrontWheelJoint = new HingeJoint(body, rfWheel, rfWheel->getPosition(), Axis(0, 0, 1) * rfWheelPose);
	bodyRightFrontWheelJoint->init(odeHandle, osgHandle, false);
	joints.push_back(bodyRightFrontWheelJoint);
	auto rfMotor = std::make_shared<VelocityControlledLocoKitMotor>(odeHandle, bodyRightFrontWheelJoint, conf.wheelMotorPower);
	rfMotor->setBaseName("right front motor");
	rfMotor->setVelovityFactor(conf.wheelMotorMaxSpeed);
	addSensor(rfMotor);
	addMotor(rfMotor);

	// Create and join right rear wheel
	auto rrRotation = Matrix::rotate(M_PI / 2.0, 0, 1, 0);
	auto rrTranslation = Matrix::translate(-((conf.bodyX/2) + conf.wheelHeight / 2.0), -conf.axelDistance/2, .0);
	Matrix rrWheelPose = rrRotation * rrTranslation * pose;
	auto rrWheel = makeWheel(rrWheelPose);
	auto bodyRightRearWheelJoint = new HingeJoint(body, rrWheel, rrWheel->getPosition(), Axis(0, 0, 1) * rrWheelPose);
	bodyRightRearWheelJoint->init(odeHandle, osgHandle, false);
	joints.push_back(bodyRightRearWheelJoint);
	auto rrMotor = std::make_shared<VelocityControlledLocoKitMotor>(odeHandle, bodyRightRearWheelJoint, conf.wheelMotorPower);
	rrMotor->setBaseName("right rear motor");
	rrMotor->setVelovityFactor(conf.wheelMotorMaxSpeed);
	addSensor(rrMotor);
	addMotor(rrMotor);

	// Create and join left front spring and leg
	auto lfSpringRotation = Matrix::rotate(M_PI / 2.0, 0, 1, 0);
	auto lfSpringTranslation = Matrix::translate(conf.springLength/2-conf.distanceWheelCenterToLeg, 0.0, (conf.legRadius + conf.wheelHeight/2));
	Matrix lfSpringPose = lfSpringRotation * lfSpringTranslation * lfWheelPose;
	auto lfSpring = makeSpring(lfSpringPose);
	auto lfLegTranslation = Matrix::translate(0.0, 0.0, conf.legHeight/2);
	Matrix lfLegPose = lfLegTranslation * lfSpringPose;
	auto lfLeg = makeLeg(lfLegPose);
	auto lfJointPos = osg::Vec3(lfWheel->getPosition().x(), lfWheel->getPosition().y(), lfWheel->getPosition().z()+conf.distanceWheelCenterToLeg);
	auto lfWheelSpringJoint = new HingeJoint(lfWheel, lfSpring, lfJointPos, Axis(1, 0, 0) * lfSpringPose);
	lfWheelSpringJoint->init(odeHandle, osgHandle, false);
	joints.push_back(lfWheelSpringJoint);
	auto lfSpringLegJoint = new SliderJoint(lfSpring, lfLeg, lfSpring->getPosition(), Axis(0, 0, 1));
	lfSpringLegJoint->init(odeHandle, osgHandle, false);
	joints.push_back(lfSpringLegJoint);
	Spring* lfSpringServo = new Spring(lfSpringLegJoint, -conf.springLength/2, 0.0, conf.springPower, conf.springDamping, 0.0, conf.springMaxVelocity);
	lfSpringServo->setBaseName("left front spring");
	springs.push_back(lfSpringServo);
	odeHandle.addIgnoredPair(lfSpring, lfLeg);

	// Create and join left rear spring and leg
	auto lrSpringRotation = Matrix::rotate(M_PI / 2.0, 0, 1, 0);
	auto lrSpringTranslation = Matrix::translate(conf.springLength/2-conf.distanceWheelCenterToLeg, 0.0, (conf.legRadius + conf.wheelHeight/2));
	Matrix lrSpringPose = lrSpringRotation * lrSpringTranslation * lrWheelPose;
	auto lrSpring = makeSpring(lrSpringPose);
	auto lrLegTranslation = Matrix::translate(0.0, 0.0, conf.legHeight/2);
	Matrix lrLegPose = lrLegTranslation * lrSpringPose;
	auto lrLeg = makeLeg(lrLegPose);
	auto lrJointPos = osg::Vec3(lrWheel->getPosition().x(), lrWheel->getPosition().y(), lrWheel->getPosition().z()+conf.distanceWheelCenterToLeg);
	auto lrWheelSpringJoint = new HingeJoint(lrWheel, lrSpring, lrJointPos, Axis(1, 0, 0) * lrSpringPose);
	lrWheelSpringJoint->init(odeHandle, osgHandle, false);
	joints.push_back(lrWheelSpringJoint);
	auto lrSpringLegJoint = new SliderJoint(lrSpring, lrLeg, lrSpring->getPosition(), Axis(0, 0, 1));
	lrSpringLegJoint->init(odeHandle, osgHandle, false);
	joints.push_back(lrSpringLegJoint);
	Spring* lrSpringServo = new Spring(lrSpringLegJoint, -conf.springLength/2, 0.0, conf.springPower, conf.springDamping, 0.0, conf.springMaxVelocity);
	lrSpringServo->setBaseName("left rear spring");
	springs.push_back(lrSpringServo);
	odeHandle.addIgnoredPair(lrSpring, lrLeg);

	// Create and join right front spring and leg
	auto rfSpringRotation = Matrix::rotate(M_PI / 2.0, 0, 1, 0);
	auto rfSpringTranslation = Matrix::translate(conf.springLength/2-conf.distanceWheelCenterToLeg, 0.0, -(conf.legRadius + conf.wheelHeight/2));
	Matrix rfSpringPose = rfSpringRotation * rfSpringTranslation * rfWheelPose;
	auto rfSpring = makeSpring(rfSpringPose);
	auto rfLegTranslation = Matrix::translate(0.0, 0.0, conf.legHeight/2);
	Matrix rfLegPose = rfLegTranslation * rfSpringPose;
	auto rfLeg = makeLeg(rfLegPose);
	auto rfJointPos = osg::Vec3(rfWheel->getPosition().x(), rfWheel->getPosition().y(), rfWheel->getPosition().z()+conf.distanceWheelCenterToLeg);
	auto rfWheelSpringJoint = new HingeJoint(rfWheel, rfSpring, rfJointPos, Axis(1, 0, 0) * rfSpringPose);
	rfWheelSpringJoint->init(odeHandle, osgHandle, false);
	joints.push_back(rfWheelSpringJoint);
	auto rfSpringLegJoint = new SliderJoint(rfSpring, rfLeg, rfSpring->getPosition(), Axis(0, 0, 1));
	rfSpringLegJoint->init(odeHandle, osgHandle, false);
	joints.push_back(rfSpringLegJoint);
	Spring* rfSpringServo = new Spring(rfSpringLegJoint, -conf.springLength/2, 0.0, conf.springPower, conf.springDamping, 0.0, conf.springMaxVelocity);
	rfSpringServo->setBaseName("right front spring");
	springs.push_back(rfSpringServo);
	odeHandle.addIgnoredPair(rfSpring, rfLeg);

	// Create and join right rear spring and leg
	auto rrSpringRotation = Matrix::rotate(M_PI / 2.0, 0, 1, 0);
	auto rrSpringTranslation = Matrix::translate(conf.springLength/2-conf.distanceWheelCenterToLeg, 0.0, -(conf.legRadius + conf.wheelHeight/2));
	Matrix rrSpringPose = rrSpringRotation * rrSpringTranslation * rrWheelPose;
	auto rrSpring = makeSpring(rrSpringPose);
	auto rrLegTranslation = Matrix::translate(0.0, 0.0, conf.legHeight/2);
	Matrix rrLegPose = rrLegTranslation * rrSpringPose;
	auto rrLeg = makeLeg(rrLegPose);
	auto rrJointPos = osg::Vec3(rrWheel->getPosition().x(), rrWheel->getPosition().y(), rrWheel->getPosition().z()+conf.distanceWheelCenterToLeg);
	auto rrWheelSpringJoint = new HingeJoint(rrWheel, rrSpring, rrJointPos, Axis(1, 0, 0) * rrSpringPose);
	rrWheelSpringJoint->init(odeHandle, osgHandle, false);
	joints.push_back(rrWheelSpringJoint);
	auto rrSpringLegJoint = new SliderJoint(rrSpring, rrLeg, rrSpring->getPosition(), Axis(0, 0, 1));
	rrSpringLegJoint->init(odeHandle, osgHandle, false);
	joints.push_back(rrSpringLegJoint);
	Spring* rrSpringServo = new Spring(rrSpringLegJoint, -conf.springLength/2, 0.0, conf.springPower, conf.springDamping, 0.0, conf.springMaxVelocity);
	rrSpringServo->setBaseName("right rear spring");
	springs.push_back(rrSpringServo);
	odeHandle.addIgnoredPair(rrSpring, rrLeg);

	// Create and join left front bearing
	Matrix lfBearingPose = Matrix::translate( 0.0, 0.0, conf.bodyToFrameDistance-conf.legHeight/2-conf.springLength/2+conf.distanceWheelCenterToLeg) * lfLegPose;
	auto lfBearing = makeBearing(lfBearingPose);
	auto lfBearingFrameJoint = new HingeJoint(lfBearing, frame, lfBearing->getPosition(), Axis(1, 0, 0) * lfBearingPose);
	lfBearingFrameJoint->init(odeHandle, osgHandle, false);
	joints.push_back(lfBearingFrameJoint);
	auto lfLegBearingJoint = new SliderJoint(lfLeg, lfBearing, lfBearing->getPosition(), Axis(0, 0, 1) * lfBearingPose);
	lfLegBearingJoint->init(odeHandle, osgHandle, false);
	joints.push_back(lfLegBearingJoint);

	// Create and join left rear bearing
	Matrix lrBearingPose = Matrix::translate( 0.0, 0.0, conf.bodyToFrameDistance-conf.legHeight/2-conf.springLength/2+conf.distanceWheelCenterToLeg) * lrLegPose;
	auto lrBearing = makeBearing(lrBearingPose);
	auto lrBearingFrameJoint = new HingeJoint(lrBearing, frame, lrBearing->getPosition(), Axis(1, 0, 0) * lrBearingPose);
	lrBearingFrameJoint->init(odeHandle, osgHandle, false);
	joints.push_back(lrBearingFrameJoint);
	auto lrLegBearingJoint = new SliderJoint(lrLeg, lrBearing, lrBearing->getPosition(), Axis(0, 0, 1) * lrBearingPose);
	lrLegBearingJoint->init(odeHandle, osgHandle, false);
	joints.push_back(lrLegBearingJoint);

	// Create and join right front bearing
	Matrix rfBearingPose = Matrix::translate( 0.0, 0.0, conf.bodyToFrameDistance-conf.legHeight/2-conf.springLength/2+conf.distanceWheelCenterToLeg) * rfLegPose;
	auto rfBearing = makeBearing(rfBearingPose);
	auto rfBearingFrameJoint = new HingeJoint(rfBearing, frame, rfBearing->getPosition(), Axis(1, 0, 0) * rfBearingPose);
	rfBearingFrameJoint->init(odeHandle, osgHandle, false);
	joints.push_back(rfBearingFrameJoint);
	auto rfLegBearingJoint = new SliderJoint(rfLeg, rfBearing, rfBearing->getPosition(), Axis(0, 0, 1) * rfBearingPose);
	rfLegBearingJoint->init(odeHandle, osgHandle, false);
	joints.push_back(rfLegBearingJoint);

	// Create and join right rear bearing
	Matrix rrBearingPose = Matrix::translate( 0.0, 0.0, conf.bodyToFrameDistance-conf.legHeight/2-conf.springLength/2+conf.distanceWheelCenterToLeg) * rrLegPose;
	auto rrBearing = makeBearing(rrBearingPose);
	auto rrBearingFrameJoint = new HingeJoint(rrBearing, frame, rrBearing->getPosition(), Axis(1, 0, 0) * rrBearingPose);
	rrBearingFrameJoint->init(odeHandle, osgHandle, false);
	joints.push_back(rrBearingFrameJoint);
	auto rrLegBearingJoint = new SliderJoint(rrLeg, rrBearing, rrBearing->getPosition(), Axis(0, 0, 1) * rrBearingPose);
	rrLegBearingJoint->init(odeHandle, osgHandle, false);
	joints.push_back(rrLegBearingJoint);

	// Create and join left front foot
	Matrix lfFootPose = Matrix::translate( 0.0, 0.0, conf.legHeight/2-conf.distanceWheelCenterToLeg) * lfLegPose;
	auto lfFoot = makeBearing(lfFootPose);
	auto lfFootLegJoint = new FixedJoint(lfFoot, lfLeg, lfFoot->getPosition());
	lfFootLegJoint->init(odeHandle, osgHandle, false);
	joints.push_back(lfFootLegJoint);
	legContactSensors[LF] = new ContactSensor(true, 65, 1.01 * conf.footRadius, false, true, Color(0,5,0));
	legContactSensors[LF]->setInitData(odeHandle, osgHandle, TRANSM(0, 0, -(0.5) * conf.footRadius));
	legContactSensors[LF]->init(lfFoot);

	// Create and join left rear foot
	Matrix lrFootPose = Matrix::translate( 0.0, 0.0, conf.legHeight/2-conf.distanceWheelCenterToLeg) * lrLegPose;
	auto lrFoot = makeBearing(lrFootPose);
	auto lrFootLegJoint = new FixedJoint(lrFoot, lrLeg, lrFoot->getPosition());
	lrFootLegJoint->init(odeHandle, osgHandle, false);
	joints.push_back(lrFootLegJoint);
	legContactSensors[LR] = new ContactSensor(true, 65, 1.01 * conf.footRadius, false, true, Color(0,5,0));
	legContactSensors[LR]->setInitData(odeHandle, osgHandle, TRANSM(0, 0, -(0.5) * conf.footRadius));
	legContactSensors[LR]->init(lrFoot);

	// Create and join right front foot
	Matrix rfFootPose = Matrix::translate( 0.0, 0.0, conf.legHeight/2-conf.distanceWheelCenterToLeg) * rfLegPose;
	auto rfFoot = makeBearing(rfFootPose);
	auto rfFootLegJoint = new FixedJoint(rfFoot, rfLeg, rfFoot->getPosition());
	rfFootLegJoint->init(odeHandle, osgHandle, false);
	joints.push_back(rfFootLegJoint);
	legContactSensors[RF] = new ContactSensor(true, 65, 1.01 * conf.footRadius, false, true, Color(0,5,0));
	legContactSensors[RF]->setInitData(odeHandle, osgHandle, TRANSM(0, 0, -(0.5) * conf.footRadius));
	legContactSensors[RF]->init(rfFoot);

	// Create and join right rear foot
	Matrix rrFootPose = Matrix::translate( 0.0, 0.0, conf.legHeight/2-conf.distanceWheelCenterToLeg) * rrLegPose;
	auto rrFoot = makeBearing(rrFootPose);
	auto rrFootLegJoint = new FixedJoint(rrFoot, rrLeg, rrFoot->getPosition());
	rrFootLegJoint->init(odeHandle, osgHandle, false);
	joints.push_back(rrFootLegJoint);
	legContactSensors[RR] = new ContactSensor(true, 65, 1.01 * conf.footRadius, false, true, Color(0,5,0));
	legContactSensors[RR]->setInitData(odeHandle, osgHandle, TRANSM(0, 0, -(0.5) * conf.footRadius));
	legContactSensors[RR]->init(rrFoot);

	// Rubber feet
	const Substance FootSubstance(3.0, 0.0, 500.0, 0.1);
	odeHandle.substance = FootSubstance;

	// Add position sensors
	startPosition = getPosition();	// MS edit
}

lpzrobots::Primitive* SpringyBot::makeBody(const Matrix& pose) {

	// Allocate object
	lpzrobots::Primitive* body = new Box(conf.bodyX, conf.bodyY, conf.bodyZ);

	// Set texture from Image library
	body->setTexture("Images/purple_velour.jpg");

	// Initialize the primitive
	body->init(odeHandle, conf.bodyMass, osgHandle);

	// Set pose
	body->setPose(pose);

	// Add to objects
	objects.push_back(body);

	return body;
}

lpzrobots::Primitive* SpringyBot::makeWheel(const osg::Matrix& pose) {

	// Allocate object
	lpzrobots::Primitive* wheel = new Cylinder(conf.wheelRadius, conf.wheelHeight);

	// Set texture from Image library
	wheel->setTexture("Images/chess.rgb");

	// Initialize the primitive
	wheel->init(odeHandle, conf.wheelMass, osgHandle);

	// Set pose
	wheel->setPose(pose);

	// Add to objects
	objects.push_back(wheel);

	return wheel;
}



lpzrobots::Primitive* SpringyBot::makeSpring(const osg::Matrix& pose) {

	// Allocate object
	lpzrobots::Primitive* spring = new Cylinder(conf.legRadius*0.9, conf.springLength);

	// Set texture from Image library
	spring->setTexture("Images/purple_velour.jpg");

	// Initialize the primitive
	spring->init(odeHandle, conf.wheelMass, osgHandle);

	// Set pose
	spring->setPose(pose);

	// Add to objects
	objects.push_back(spring);

	return spring;
}

lpzrobots::Primitive* SpringyBot::makeLeg(const osg::Matrix& pose) {

	// Allocate object
	lpzrobots::Primitive* leg = new Cylinder(conf.legRadius, conf.legHeight-conf.springLength);

	// Set texture from Image library
	leg->setTexture("Images/chess.rgb");

	// Initialize the primitive
	leg->init(odeHandle, conf.wheelMass, osgHandle);

	// Set pose
	leg->setPose(pose);

	// Add to objects
	objects.push_back(leg);

	return leg;
}

lpzrobots::Primitive* SpringyBot::makeBearing(const osg::Matrix& pose) {

	// Allocate object
	lpzrobots::Primitive* bearing = new Sphere(conf.wheelHeight+conf.legRadius);

	// Set texture from Image library
	bearing->setTexture("Images/purple_velour.jpg");

	// Initialize the primitive
	bearing->init(odeHandle, conf.wheelMass, osgHandle);

	// Set pose
	bearing->setPose(pose);

	// Add to objects
	objects.push_back(bearing);

	return bearing;
}

lpzrobots::Primitive* SpringyBot::makeFoot(const osg::Matrix& pose) {

	// Allocate object
	lpzrobots::Primitive* foot = new Sphere(conf.footRadius);

	// Set texture from Image library
	foot->setTexture("Images/purple_velour.jpg");

	// Initialize the primitive
	foot->init(odeHandle, conf.wheelMass, osgHandle);

	// Set pose
	foot->setPose(pose);

	// Add to objects
	objects.push_back(foot);

	return foot;
}

void SpringyBot::doInternalStuff(GlobalData& globalData) {
	for (SpringList::iterator it = springs.begin(); it != springs.end(); it++) {
		(*it)->set(0.0);
	}

	writeFootContactsToFile();
}

void SpringyBot::update() {
	OdeRobot::update();

	for (int i = 0; i < LEG_POS_MAX; i++) {
		if (legContactSensors[LegPos(i)])
			legContactSensors[LegPos(i)]->update();
	}
}

void SpringyBot::sense(GlobalData& globalData) {

	for (int i = 0; i < LEG_POS_MAX; i++) {
		if (legContactSensors[LegPos(i)])
			legContactSensors[LegPos(i)]->sense(globalData);
	}
}

void SpringyBot::writeFootContactsToFile(void) {
	if (outputFile.is_open())
	{
		for (int i = 0; i < LEG_POS_MAX; i++) {
			if (legContactSensors[LegPos(i)])
				outputFile << legContactSensors[LegPos(i)]->get() << ",";
		}
		outputFile << std::endl;
	}
	else
		std::cout << "File not open" << std::endl;
}


} /* namespace lpzrobots */
