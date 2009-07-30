/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
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
 *  DESCRIPTION                                                            *
 *  
 *                                                                         *
 *                                                                         *
 *  $Log$
 *  Revision 1.1  2009-07-30 11:52:32  guettler
 *  new CameraHandle replacing static variables in the CameraManipulators
 *										   *
 *                                                                         *
 **************************************************************************/

#include "camerahandle.h"


namespace lpzrobots
{
  CameraHandle::CameraHandle() : eye(0,0,0), home_eye(0,0,0), home_view(0,0,0), desiredEye(0,0,0), desiredView(0,0,0), home_externally_set(false),
      watchingAgent(0), watchingAgentDefined(false), oldPositionOfAgent(0,0,0), oldPositionOfAgentDefined(false),
      doManipulation(No), manipulationPoint(0,0,0), manipulationViz(0)
  {
  }

  CameraHandle::~CameraHandle()
  {
  }


}
