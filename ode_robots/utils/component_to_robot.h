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
