/***************************************************************************
 *  callback class for motion blur                                         *
 *  the param globalData.odeConfig.motionPersistence determines            *
 *  the level of motion blur                                               *
 *                                                                         *
 ***************************************************************************/
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
 *   Revision 1.4  2008-09-11 15:24:01  martius
 *   motioncallback resurrected
 *   noContact substance
 *   use slider center of the connecting objects for slider drawing
 *
 *   Revision 1.2  2006/07/14 12:23:35  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.1  2006/04/27 16:18:24  robot3
 *   motionblurcallback for the simulation
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __MOTIONBLURCALLBACK_H
#define __MOTIONBLURCALLBACK_H

#include <osgViewer/Viewer>
#include "globaldata.h"

namespace lpzrobots {
  
  /** a class that enables motion blur for the scenegraph 
   *  should be called in the main simulation loop
   */
  class MotionBlurOperation: public osg::Operation
  {
  public:
    /** globalData.odeConfig.motionPersistence - determines the level of motion blur,
     *  between 0.0 and 1.0, for example:
     *  heavy motion blur is set by globalData.odeConfig.motionPersistance=0.25
     *  light motuib blur is set by globalData.odeConfig.motionPersistence=0.1
     */ 
    MotionBlurOperation(GlobalData& global):
      osg::Operation("MotionBlur",true),
      cleared_(false),
      persistence_(global.odeConfig.motionPersistence)
    {
    }
    
    virtual void operator () (osg::Object* object);

  private:
    bool cleared_;
    double t0_;
    double persistence_;
  };

} // namespace


#endif
