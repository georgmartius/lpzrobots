/*
 * XMLRobot.h
 *
 *  Created on: 03.02.2010
 *      Author: robot3
 */

#ifndef XMLROBOT_H_
#define XMLROBOT_H_

#include <ode_robots/oderobot.h>
#include "XercescForwardDecl.h"
#include "XMLObject.h"


class XMLRobot: public lpzrobots::OdeRobot, public XMLObject {
public:
        XMLRobot(XERCESC::DOMNode* robotNode, XMLParserEngine& xmlEngine, const std::string& name);
        virtual ~XMLRobot();


           /// update the OSG notes here
            virtual void update();

            /** sets the pose of the vehicle
                @param pose desired 4x4 pose matrix
            */
            virtual void place(const osg::Matrix& pose);

            /** this function is called in each timestep after control. It
                should perform robot-internal checks and actions,
                like acting and sensing of internal motors/sensors etc.
                @param globalData structure that contains global data from the simulation environment
            */
            virtual void doInternalStuff(lpzrobots::GlobalData& globalData);

            /// return the primitive of the robot that is used for tracking and camera following
            virtual lpzrobots::Primitive* getMainPrimitive() const;

};

#endif /* XMLROBOT_H_ */
