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
#ifndef __GRABFRAME_H
#define __GRABFRAME_H

#include <string>
#include <osg/Image>
#include <osg/Camera>
#include <selforg/backcaller.h>

namespace lpzrobots{

  class VideoStream : public osg::Camera::DrawCallback, public BackCaller {
  public:
    static const BackCaller::CallbackableType FRAMECAPTURE = 898989;
    VideoStream(){ opened=false; pause = false;}

    void open(const std::string& dir, const std::string& filename);
    void close();
    bool grabAndWriteFrame(const osg::Camera& camera);

    bool isOpen() const { return opened; }

    // DrawCallback interface
    virtual void operator() (const osg::Camera &) const ;

    virtual long int getCounter() { return counter; }
    virtual const std::string& getDirectory() { return directory; }

    bool pause;
  private:
    bool opened;
    std::string filename;
    std::string directory;
    unsigned int w;
    unsigned int h;
    long int counter;
  };

}

#endif
