/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
#ifndef __CAMERAMANIPULATOR_H
#define __CAMERAMANIPULATOR_H

#include "osgforwarddecl.h"

#include <osg/Version>
#if OPENSCENEGRAPH_MAJOR_VERSION >= 3
#include <osgGA/CameraManipulator>
#define OSGCameraManipulator osgGA::CameraManipulator
#else
#include <osgGA/MatrixManipulator>
#define OSGCameraManipulator osgGA::MatrixManipulator
#endif
#include "globaldata.h"
#include <selforg/position.h>
#include <selforg/callbackable.h>
#include "camerahandle.h"

namespace lpzrobots {
  // forward declaration
  class OSGPrimitive;

  /**
     CameraManipulator is a MatrixManipulator which provides a flying camera
     updating of the camera position & orientation.
     Left mouse button: Pan and tilt
     Right mouse button: forward and sideways
     Middle mouse button: up and sideways

     It also enables to manipulate agents with forces
  */
  class CameraManipulator : public OSGCameraManipulator, public Callbackable
    {
    public:

      CameraManipulator(osg::Node* node, GlobalData& global, CameraHandle& cameraHandle);


      /** returns the classname of the manipulator
          it's NECCESSARY to define this funtion, otherwise
          the new manipulator WON'T WORK! (but ask me not why)
      */
      virtual const char* className() const { return "Default Camera"; }

      /** set the position of the matrix manipulator using a 4x4 Matrix.*/
      virtual void setByMatrix(const osg::Matrixd& matrix);

      /** set the position of the matrix manipulator using a 4x4 Matrix.*/
      virtual void setByInverseMatrix(const osg::Matrixd& matrix) {
        setByMatrix(osg::Matrixd::inverse(matrix));
      }

      /** get the position of the manipulator as 4x4 Matrix.*/
      virtual osg::Matrixd getMatrix() const;

      /** get the position of the manipulator as a inverse matrix of the manipulator,
          typically used as a model view matrix.*/
      virtual osg::Matrixd getInverseMatrix() const;


      /**
       * is called every time the draw is updated. computes the
       * movement of the camera, which is a difference between
       * the desired pos and view and the actual pos and view.
       */
      /*
        virtual void computeMovement();*/

      virtual void setNode(osg::Node*);

      virtual const osg::Node* getNode() const;

      virtual osg::Node* getNode();

      /// set the home position of the camera. (and place it there)
      virtual void setHome(const osg::Vec3& eye, const osg::Vec3& view);

      /// place the camera at its home position
      virtual void home(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us);

      virtual void init(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us);

      virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us);

      /** Get the keyboard and mouse usage of this manipulator.*/
      virtual void getUsage(osg::ApplicationUsage& usage) const;

      /** updates the camera module at every drawstep
          should be called from the simulation loop
      */
      virtual void update();

      /** manipulate agent if Manipulation is active
          (should be called every simulation step)
      */
      virtual void manipulateAgent( OsgHandle& osgHandle);

      /**
       * Sets the agent to be watched with the camera.
       * @param agent to set
       */
      virtual void setWatchedAgent(OdeAgent* agent);

      /// returns watched agent
      virtual OdeAgent* getWatchedAgent();

      /// called if agents list changed
      virtual void doOnCallBack(BackCaller* source, BackCaller::CallbackableType type
                                = BackCaller::DEFAULT_CALLBACKABLE_TYPE);

    protected:

      virtual ~CameraManipulator();

      /** Reset the internal GUIEvent stack.*/
      virtual void flushMouseEventStack();
      /** Add the current mouse GUIEvent to internal stack.*/
      virtual void addMouseEvent(const osgGA::GUIEventAdapter& ea);

      virtual void computeMatrix();

      /** For the give mouse movement calculate the movement of the camera.
          Return true is camera has moved and a redraw is required.*/
      virtual bool calcMovement();

      /**
       * Checks if an agent is selected and if this agent is available.
       * This agent must be listed in the global agent list.
       * @return true if defined, otherwise false
       */
      virtual bool isWatchingAgentDefined();

      // Internal event stack comprising last three mouse events.
      osg::ref_ptr<const osgGA::GUIEventAdapter> event_old;
      osg::ref_ptr<const osgGA::GUIEventAdapter> event;

      osg::ref_ptr<osg::Node> node;

      float modelScale;
      osg::Matrixd  pose;  // complete pose (updated by computeMatrix()

      CameraHandle& camHandle;

      GlobalData& globalData; // the global environment variables


      double degreeSmoothness; // smoothness factor for the view
      double lengthSmoothness; // smoothness factor for the eye
      double degreeAccuracy; // accuracy factor for the view-smoothness
      double lengthAccuracy; // accuracy factor for the eye-smoothness


      /** This manages the robots, switching between them and so on
          Is normally called from handle(...)
      */
      virtual void manageAgents(const int& fkey);


      /** This handles robot movements, so that the camera movemenent is right affected.
          should normally be overwritten by new cameramanipulator
      */
      virtual void calcMovementByAgent();


      /** Sets the right view and eye if the robot has changed.
          Is called from manageRobots();
          should be overwritten by new cameramanipulator (if needed)
      */
      virtual void setHomeViewByAgent();
      virtual void setHomeEyeByAgent();


      /** moves behind the robot which is actually watched
       */
      virtual void moveBehindAgent();

      /** centers on the robot which is actually watched
       */
      virtual void centerOnAgent();

      /** manipulates Agent by forces. The given points are screen coords (-1 to 1) normalized.
      */
      virtual void calcManipulationPointHorizontal(float x, float y);

      virtual void calcManipulationPointVertical(float x, float y);

      virtual void calcManipulationPoint(float x, float y);


    };

}

#endif
