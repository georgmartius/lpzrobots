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
 *   Revision 1.4  2005-11-09 13:31:51  martius
 *   GPL'ised
 *
 ***************************************************************************/
#ifndef COMPONENT_TO_ROBOT_H
#define COMPONENT_TO_ROBOT_H

#include "component.h"
#include "abstractrobot.h"


namespace university_of_leipzig {
namespace robots {
  

class ComponentToRobot : public AbstractRobot
{
 protected:
  IComponent *p_component;

  WireContainer wire_container;

 public:
  ComponentToRobot(IComponent *_p_component, const OdeHandle& odehandle);
  virtual ~ComponentToRobot();

  const char*     getName() const;

  virtual void     draw               ();
  virtual void     place              (Position pos , Color *c = NULL);
  virtual bool     collisionCallback  (void *data, dGeomID o1, dGeomID o2);
  virtual void     doInternalStuff    (const GlobalData& globalData);
  virtual int      getSensors         (sensor* sensors, int sensornumber);
  virtual void     setMotors          (const motor* motors, int motornumber);
  virtual int      getSensorNumber    ();
  virtual int      getMotorNumber     ();
  virtual Position getPosition        ();
  virtual int      getSegmentsPosition(vector<Position> &poslist);
  virtual void     setColor           (Color col);

};


}
}


#endif
