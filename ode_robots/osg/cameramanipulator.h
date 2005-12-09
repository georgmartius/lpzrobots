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
 ***************************************************************************
 *                                                                         *
 *  Camera Manipulation by mouse and keyboard                              *
 *                                                                         *
 *   $Log$
 *   Revision 1.1.2.1  2005-12-09 16:56:21  martius
 *   camera is working now
 *
 *   Revision 1.1.2.1  2005/12/06 17:38:21  martius
 *   *** empty log message ***
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __CAMERAMANIPULATOR_H
#define __CAMERAMANIPULATOR_H

#include <osgGA/MatrixManipulator>
#include <osg/Quat>

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

    CameraManipulator(osg::Node* node);

    virtual const char* className() const { return "Camera"; }

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


    virtual void setNode(osg::Node*);

    virtual const osg::Node* getNode() const;

    virtual osg::Node* getNode();

    virtual void home(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us);

    virtual void init(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us);

    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us);

    /** Get the keyboard and mouse usage of this manipulator.*/
    virtual void getUsage(osg::ApplicationUsage& usage) const;

  protected:

    virtual ~CameraManipulator();

    /** Reset the internal GUIEvent stack.*/
    void flushMouseEventStack();
    /** Add the current mouse GUIEvent to internal stack.*/
    void addMouseEvent(const osgGA::GUIEventAdapter& ea);

    void computeMatrix();

    /** For the give mouse movement calculate the movement of the camera.
	Return true is camera has moved and a redraw is required.*/
    bool calcMovement();

    // Internal event stack comprising last three mouse events.
    osg::ref_ptr<const osgGA::GUIEventAdapter> event_old;
    osg::ref_ptr<const osgGA::GUIEventAdapter> event;

    osg::ref_ptr<osg::Node> node;

    float modelScale;
        
    osg::Vec3   eye;      // position of the camera
    osg::Vec3   view;     // view angles in degree (pan, tilt, yaw)
    
    osg::Matrixd  pose;  // complete pose (updated by computeMatrix()

  };



}

#endif
