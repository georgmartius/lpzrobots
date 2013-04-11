#include "multi_controller.h"

void (*cameraHandlingFunction)();

MultiController::MultiController()
{
  it_active_controller = controller_container.begin();
}


void MultiController::init(int sensornumber, int motornumber)
{
  (*it_active_controller)->init(sensornumber, motornumber);
}


AbstractController::paramkey MultiController::getName() const
{
  return (*it_active_controller)->getName();
}


int MultiController::getSensorNumber() const
{
  return (*it_active_controller)->getSensorNumber();
}


int MultiController::getMotorNumber() const
{
  return (*it_active_controller)->getMotorNumber();
}


void MultiController::step(const sensor* sensors, int sensornumber,
                           motor* motors, int motornumber)
{
  (*it_active_controller)->step(sensors, sensornumber, motors, motornumber);
}


void MultiController::stepNoLearning(const sensor* sensors,
                                     int sensornumber,
                                     motor* motors,
                                     int motornumber)
{
  (*it_active_controller)->stepNoLearning(sensors,
                                          sensornumber,
                                          motors,
                                          motornumber);
}


list<AbstractController::iparamkey>
MultiController::getInternalParamNames() const
{
  return (*it_active_controller)->getInternalParamNames();
}


std::list<AbstractController::iparamval>
MultiController::getInternalParams() const
{
  return (*it_active_controller)->getInternalParams();
}


AbstractController::paramval
MultiController::getParam(const paramkey& key, bool traverseChildren) const
{
  return (*it_active_controller)->getParam(key);
}


bool MultiController::setParam(const paramkey& key, paramval val, bool traverseChildren)
{
  return (*it_active_controller)->setParam(key, val);
}


AbstractController::paramlist MultiController::getParamList() const
{
  return (*it_active_controller)->getParamList();
}


ControllerContainer& MultiController::get_controller_container()
{
  return controller_container;
}


void MultiController::set_active_controller(ControllerContainer::iterator it)
{
  it_active_controller = it;
}


ControllerContainer::iterator MultiController::get_active_controller()
{
  return it_active_controller;
}


void MultiController::next_controller()
{
  ++it_active_controller;
  if(controller_container.end() == it_active_controller)
    it_active_controller = controller_container.begin();
}
