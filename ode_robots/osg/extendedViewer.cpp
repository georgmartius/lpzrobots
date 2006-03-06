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
 *   Revision 1.1.2.1  2006-03-06 16:52:02  robot3
 *   written an own viewer because getCurrentCameraManipulator() was not implemented
 *
 *                                                                         *
 ***************************************************************************/
/*
#include <osg/LightSource>
#include <osg/ApplicationUsage>
#include <osg/AlphaFunc>
#include <osg/io_utils>
#include <osgUtil/UpdateVisitor>

#include <osgDB/Registry>

#include <osgGA/EventVisitor>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/UFOManipulator>
#include <osgGA/StateSetManipulator>
*/
#include "extendedViewer.h"
/*
#include <osgProducer/ViewerEventHandler>

#include <stdio.h>
*/
using namespace Producer;
using namespace osgProducer;
using namespace osg;
using namespace lpzrobots;

#ifdef __APPLE__
#define SINGLE_THREAD_KEYBOARDMOUSE
#endif

ExtendedViewer::ExtendedViewer(): Viewer() {}

ExtendedViewer::ExtendedViewer(Producer::CameraConfig *cfg): Viewer(cfg) {}

ExtendedViewer::ExtendedViewer(const std::string& configFile): Viewer(configFile) {}

ExtendedViewer::ExtendedViewer(osg::ArgumentParser& arguments): Viewer(arguments) {}

ExtendedViewer::~ExtendedViewer() {}



