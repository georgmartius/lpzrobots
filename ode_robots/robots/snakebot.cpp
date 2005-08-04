#include "snakebot.h"


namespace university_of_leipzig {
namespace robot {


/*****************************************************************************/
/* SnakeBot                                                                  */
/*****************************************************************************/
// a robot consisting simply of a robot arm component

SnakeBot::SnakeBot(const RobotArmDescription &r_desc) :
  AbstractRobot(r_desc.p_world->get_world_id(),
		r_desc.p_world->get_space_id(),
		r_desc.p_world->get_joint_group_id_contact()),
  ccu_robot_arm_component(r_desc)
{
  ccu_robot_arm_component.expose_wires(wire_container);

  std::cout << "wire count = " << wire_container.size() << "\n";


  sensorfeld.resize(wire_container.size());

  std::vector<Sensor>::iterator it = sensorfeld.begin();
  for(it; sensorfeld.end() != it; ++it)
    memset(&(*it), 0, sizeof(Sensor));

  r_desc.p_world->register_component(ccu_robot_arm_component);

}
/*
SnakeBot::~SnakeBot()
{
  //  p_world->unregister_component(ccu_robot_arm_component);
}
*/

void SnakeBot::draw()
{
  ccu_robot_arm_component.draw();
}



void SnakeBot::place(Position pos, Color *c)
{
  return;
}


void SnakeBot::getWinkelDifferenz ( int motor , double* X )
{
  //Eingangsdaten sind die Winkeldifferenzen eines Roboterberechnungsschrittes
  //Abfangen des Winkeldifferenzsprunges bei ueberschreiten er 2PI-Marke
  //tritt auf wenn die -PI- oder die PI-Marke ueberschritten wird
  
  double w = sensorfeld[motor].istwinkel - sensorfeld[motor].istwinkel_alt;
  
  if ( ( w < -M_PI )
       || ( w >  M_PI ) )
    {
      //1. Fall(PI-Marke wird ueberschritten, also annaeherung vom Positiven) -> es ergibt sich eine negative Winkeldifferenz
      if ( w > M_PI )
	{
	  *X = -(2*M_PI - w);
	  //dsPrint ( "%lf  %lf => %lf \n" , sensorfeld[motor].istwinkel ,  sensorfeld[motor].istwinkel_alt , -(2*M_PI-w)/M_PI );
	}
      //2. Fall(-PI-Marke wird ueberschritten, also Annaeherung vom Negativen) -> es ergibt sich eine positive Winkeldifferenz
      if ( w < -M_PI )
	{
	  *X = (2*M_PI + w);
	  //dsPrint ( "%lf  %lf => %lf \n" , sensorfeld[motor].istwinkel ,  sensorfeld[motor].istwinkel_alt , (2*M_PI+w)/M_PI );
	}
    }
  else
    {
      *X = w;
      //dsPrint ( "%lf  %lf = %lf \n" , sensorfeld[motor].istwinkel ,  sensorfeld[motor].istwinkel_alt , w/M_PI );
    }
}


int SnakeBot::getSensors(sensor* sensors, int sensornumber)
{
  sensoraktualisierung ();
  for ( int n = 0; n < sensornumber; n++ ) {
    if(0) // AUSGABEART
      (*sensors++) = sensorfeld[n].istwinkel;
    else
	getWinkelDifferenz ( n , sensors++ );
    }
  
  return sensorfeld.size(); //es sind immer alle Sensorwerte durchgeschrieben, da  alle in einem Schritt aktualisiert werden 
}

void SnakeBot::sensoraktualisierung()
{

  WireContainer::const_iterator it = wire_container.begin();
  for (int n = 0; n < sensorfeld.size(); n++) {

    const IOutputWire *p_output_wire = dynamic_cast<const IOutputWire*>(*it);
    if(NULL == p_output_wire)
      std::cout << "Xoops... wtf...?\n";
    

    sensorfeld[n].istwinkel_alt = sensorfeld[n].istwinkel;

    sensorfeld[n].istwinkel = p_output_wire->get();

    ++it;
  }

}


void SnakeBot::setMotors(const motor* motors, int motornumber)
{
  WireContainer::iterator it = wire_container.begin();
  for(int i = 0; i < motornumber; ++i) {
    //    std::cout << "MV = " << *motors << "\n";
    if(*motors < -1.0 || *motors > 1.0) {
      std::cerr << "invalid motor value\n";
      exit(-1);

    }

    IInputWire *p_input_wire = dynamic_cast<IInputWire*>(*it);
    if(NULL == p_input_wire)
      std::cout << "Yoops... wtf...\n";

    p_input_wire->put(*(motors++));

    ++it;
  }
}




int SnakeBot::getSensorNumber()
{
  return wire_container.size();
}


int SnakeBot::getMotorNumber()
{
  return wire_container.size();
}


Position SnakeBot::getPosition()
{
  return Position();
}


int SnakeBot::getSegmentsPosition(vector<Position> &poslist)
{
  return 0;
};


}
}
