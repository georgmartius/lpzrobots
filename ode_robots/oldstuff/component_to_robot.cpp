/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Rald Der       <ralfder at mis dot mpg dot de>                       *
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
#include "component_to_robot.h"

namespace lpzrobots {


ComponentToRobot::ComponentToRobot(IComponent *_p_component,
                                   const OdeHandle &odehandle,
                                   const OsgHandle &osghandle) :
  OdeRobot(odeHandle, osgHandle),
  p_component(_p_component)
{
  if(NULL == p_component)
    InvalidArgumentException().raise();

  _p_component->expose_wires(wire_container);
}


ComponentToRobot::~ComponentToRobot()
{
}


const char* ComponentToRobot::getName() const
{
  return "ComponentToRobot";
}


void ComponentToRobot::draw()
{
  p_component->draw();
}


void ComponentToRobot::place(Position pos , Color *c)
{
  // not implemented
  // IPlaceable *p_placeable = dynamic_cast<IPlaceable>(p_component);
}


bool ComponentToRobot::collisionCallback(void *data, dGeomID o1, dGeomID o2)
{
  return p_component->collision_callback(NULL, o1, o2);
}

void ComponentToRobot::doInternalStuff(GlobalData& globalData){
  // todo reset sensors
}


int ComponentToRobot::getSensors(sensor* sensors, int sensornumber)
{
  WireContainer::iterator it_wire = wire_container.begin();

  int current_sensor = 0;

  for(int i = 0; i < sensornumber; ++i) {

    IOutputWire *p_output_wire = dynamic_cast<IOutputWire*>(*it_wire);

    if(NULL == p_output_wire)
      continue;

    if(wire_container.end() == it_wire)
      IndexOutOfBoundsException().raise();

    sensors[current_sensor] = p_output_wire->get();

    ++it_wire;
    ++current_sensor;
  }

  return current_sensor;
}


void ComponentToRobot::setMotors(const motor* motors, int motornumber)
{
  int current_motor_count = 0;

  WireContainer::iterator it = wire_container.begin();
  for(int i = 0; i < motornumber; ++i) {
    //    std::cout << "MV = " << *motors << "\n";
    if(*motors < -1.0 || *motors > 1.0) {
      std::cerr << "invalid motor value\n";
      exit(-1);

    }

    IInputWire *p_input_wire = dynamic_cast<IInputWire*>(*it);
    if(NULL == p_input_wire)
      continue;


    if(wire_container.end() == it)
      IndexOutOfBoundsException().raise();

    ++current_motor_count;
    p_input_wire->put(*(motors++));

    ++it;
  }
}


int ComponentToRobot::getSensorNumber()
{
  WireContainer::const_iterator it_wire = wire_container.begin();

  unsigned output_wire_count = 0;

  while(wire_container.end() != it_wire) {
    if(NULL != dynamic_cast<const IOutputWire*>(*(it_wire++)))
      ++output_wire_count;
  }

  return output_wire_count;
}


int ComponentToRobot::getMotorNumber()
{
  WireContainer::const_iterator it_wire = wire_container.begin();

  unsigned input_wire_count = 0;

  while(wire_container.end() != it_wire) {
    if(NULL != dynamic_cast<const IInputWire*>(*(it_wire++)))
      ++input_wire_count;
  }

  return input_wire_count;
}

int ComponentToRobot::getSegmentsPosition(vector<Position> &poslist)
{
  return 0;
}


void ComponentToRobot::setColor(Color col)
{
  // not implemented
}

Primitive* ComponentToRobot::getMainPrimitive(){
  return 0; // FIXME get object from first component
}



}
