#ifdef FUCKTHECODE
#include "abstractrobot.h"
#include "component.h"
#include "world.h"

namespace university_of_leipzig {
namespace robot {


class SnakeBot : public AbstractRobot {
  CCURobotArmComponent ccu_robot_arm_component;

  WireContainer wire_container;
  

  typedef struct
  {
    double istwinkel;
    double istwinkel_alt;
    double sollwinkel;
    
    double x;
    double y;
    double z;
  } Sensor;
  

  std::vector<Sensor> sensorfeld;


 public:
  SnakeBot(const RobotArmDescription &r_desc);

  void draw();

  void place(Position pos, Color *c = 0);

  int getSensors(sensor* sensors, int sensornumber);


  void setMotors(const motor* motors, int motornumber);

  void sensoraktualisierung();
  int getSensorNumber();
  int getMotorNumber();
  Position getPosition();
  int getSegmentsPosition(vector<Position> &poslist);

  void getWinkelDifferenz ( int motor , double* X );
};


}
}

#endif
