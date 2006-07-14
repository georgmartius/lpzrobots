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
 *                                                                         *
 *   $Log$
 *   Revision 1.13  2006-07-14 11:57:23  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.12.4.8  2006/06/29 16:32:44  robot3
 *   implementation of odeconfig moved to odeconfig.cpp
 *
 *   Revision 1.12.4.7  2006/05/08 12:12:36  robot3
 *   changes from Revision 1.40.4.27 reverted
 *
 *   Revision 1.12.4.6  2006/04/27 16:17:38  robot3
 *   implemented some functions for motionblurcallback
 *
 *   Revision 1.12.4.5  2006/03/28 09:55:12  robot3
 *   -main: fixed snake explosion bug
 *   -odeconfig.h: inserted cameraspeed
 *   -camermanipulator.cpp: fixed setbyMatrix,
 *    updateFactor
 *
 *   Revision 1.12.4.4  2006/02/08 16:14:28  martius
 *   no namespace using
 *
 *   Revision 1.12.4.3  2006/01/10 15:08:15  martius
 *   controlinterval is 1 by default
 *
 *   Revision 1.12.4.2  2005/12/06 10:13:23  martius
 *   openscenegraph integration started
 *
 *   Revision 1.12.4.1  2005/11/14 17:37:00  martius
 *   changed makefile structure to have and include directory
 *   mode to selforg
 *
 *   Revision 1.12  2005/11/09 13:29:19  martius
 *   GPL'ised
 *
 ***************************************************************************/
#ifndef __ODECONFIG_H
#define __ODECONFIG_H

#include <selforg/configurable.h>
#include "odehandle.h"

namespace lpzrobots {

  class OdeConfig : public Configurable {
  public:
    
    // creates new instance of OdeConfig with default values
    OdeConfig();
    
    ~OdeConfig() {}
    
    virtual paramkey getName() const;
    
    virtual paramlist getParamList() const;

    virtual paramval getParam(const paramkey& key) const;
        
    virtual bool setParam(const paramkey& key, paramval val);

    virtual void setOdeHandle(const OdeHandle& odeHandle);

    virtual void setVideoRecordingMode(bool mode);

  private:
    /// calculates the draw interval with simStepSize and realTimeFactor so that we have 25 frames/sec
    virtual int calcDrawInterval25();

    /// calculates the draw interval with simStepSize and realTimeFactor so that we have 50 frames/sec
    /// this is much better for graphical visualization (smoother)
    virtual int calcDrawInterval50();


  public:
    bool videoRecordingMode;
    double simStepSize;
    double realTimeFactor;
    int drawInterval;
    int controlInterval;
    double motionPersistence;
    double noise;
    double gravity;
    double cameraSpeed;
    bool drawBoundings;
    std::string name;
    OdeHandle odeHandle;
  };

}

#endif
