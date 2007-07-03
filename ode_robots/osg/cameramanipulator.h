/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 ***************************************************************************
 *                                                                         *
 *  Camera Manipulation by mouse and keyboard                              *
 *                                                                         *
 *   $Log$
 *   Revision 1.3  2007-07-03 13:15:17  martius
 *   odehandle.h in cpp files included
 *
 *   Revision 1.2  2006/07/14 12:23:34  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.12  2006/06/29 16:35:56  robot3
 *   includes cleared up
 *
 *   Revision 1.1.2.11  2006/05/29 20:00:33  robot3
 *   added pos1 (center on agent) and end (move behind agent)
 *
 *   Revision 1.1.2.10  2006/03/19 13:32:12  robot3
 *   race mode now works
 *
 *   Revision 1.1.2.9  2006/03/08 13:19:13  robot3
 *   basic modifications, follow mode now works
 *
 *   Revision 1.1.2.8  2006/03/06 16:57:01  robot3
 *   -more stable version
 *   -code optimized
 *   -some static variables used by all cameramanipulators
 *
 *   Revision 1.1.2.7  2006/03/05 15:02:24  robot3
 *   camera moves now smooth
 *
 *   Revision 1.1.2.6  2006/03/04 15:04:33  robot3
 *   cameramanipulator is now updated with every draw intervall
 *
 *   Revision 1.1.2.5  2006/03/03 12:08:50  robot3
 *   preparations made for new cameramanipulators
 *
 *   Revision 1.1.2.4  2006/02/01 10:24:34  robot3
 *   new camera manipulator added
 *
 *   Revision 1.1.2.3  2005/12/29 12:56:12  martius
 *   setHome
 *
 *   Revision 1.1.2.2  2005/12/15 17:03:42  martius
 *   cameramanupulator setPose is working
 *   joints have setter and getter parameters
 *   Primitives are not longer inherited from OSGPrimitive, moreover
 *   they aggregate them
 *
 *   Revision 1.1.2.1  2005/12/09 16:56:21  martius
 *   camera is working now
 *
 *   Revision 1.1.2.1  2005/12/06 17:38:21  martius
 *   *** empty log message ***
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __CAMERAMANIPULATOR_H
#define __CAMERAMANIPULATOR_H

#include "osgforwarddecl.h"
#include <osgGA/MatrixManipulator>
#include "globaldata.h"
#include <selforg/position.h>

namespace lpzrobots {

  /**
     CameraManipulator is a MatrixManipulator which provides Flying simulator-like
     updating of the camera position & orientation. 
     Left mouse button: Pan and tilt
     Right mouse button: forward and sideways
     Middle mouse button: up and sideways
  */

  class CameraManipulator : public osgGA::MatrixManipulator
    {
    public:

      CameraManipulator(osg::Node* node, GlobalData& global);


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

      // Internal event stack comprising last three mouse events.
      osg::ref_ptr<const osgGA::GUIEventAdapter> event_old;
      osg::ref_ptr<const osgGA::GUIEventAdapter> event;

      osg::ref_ptr<osg::Node> node;

      float modelScale;
      osg::Matrixd  pose;  // complete pose (updated by computeMatrix()
        
      static osg::Vec3   eye;      // position of the camera
      static osg::Vec3   view;     // view angles in degree (pan, tilt, yaw)
      static osg::Vec3   home_eye;  // home position of the camera
      static osg::Vec3   home_view; // home view angles in degree (pan, tilt, yaw)
      static bool home_externally_set;
    
      static osg::Vec3   desiredEye;      // desired position of the camera
      static osg::Vec3   desiredView;     // desired view angles in degree (pan, tilt, yaw)

      static OdeAgent* watchingAgent; // the robot which is actually watched
      static bool watchingAgentDefined;

      static Position oldPositionOfAgent; // because the return of getSpeed() seems not to be useful
      static bool oldPositionOfAgentDefined;
    
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
      
      static int i;

  
    };

}

#endif
