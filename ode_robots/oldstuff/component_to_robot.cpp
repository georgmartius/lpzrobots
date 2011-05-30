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
 *   Revision 1.1  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.3  2007/11/07 13:13:29  martius
 *   changed doInternalStuff
 *
 *   Revision 1.2  2006/07/14 12:23:31  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.1  2006/01/12 14:52:37  martius
 *   moved here from util directory
 *
 *   Revision 1.4.4.3  2005/12/06 10:13:26  martius
 *   openscenegraph integration started
 *
 *   Revision 1.4.4.2  2005/11/15 12:30:24  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.4.4.1  2005/11/14 17:37:25  martius
 *   moved to selforg
 *
 *   Revision 1.4  2005/11/09 13:31:51  martius
 *   GPL'ised
 *
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
