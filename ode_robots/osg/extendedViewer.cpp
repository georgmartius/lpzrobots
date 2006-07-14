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
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2006-07-14 12:23:34  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.2  2006/06/29 16:35:56  robot3
 *   includes cleared up
 *
 *   Revision 1.1.2.1  2006/03/06 16:52:02  robot3
 *   written an own viewer because getCurrentCameraManipulator() was not implemented
 *
 *                                                                         *
 ***************************************************************************/
#include "extendedViewer.h"

namespace lpzrobots {

  ExtendedViewer::ExtendedViewer(): Viewer() {}

  ExtendedViewer::ExtendedViewer(Producer::CameraConfig *cfg): Viewer(cfg) {}

  ExtendedViewer::ExtendedViewer(const std::string& configFile): Viewer(configFile) {}

  ExtendedViewer::ExtendedViewer(osg::ArgumentParser& arguments): Viewer(arguments) {}

  ExtendedViewer::~ExtendedViewer() {}

  osgGA::MatrixManipulator* ExtendedViewer::getCurrentCameraManipulator() {
    return _keyswitchManipulator->getCurrentMatrixManipulator();
  }

}
