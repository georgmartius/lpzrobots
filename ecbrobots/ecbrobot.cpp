/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    marcoeckert@web.de                                                   *
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
 *   Revision 1.5  2008-08-12 11:44:42  guettler
 *   plug and play update, added some features for the ECBRobotGUI
 *
 *   Revision 1.4  2008/04/11 06:31:16  guettler
 *   Included all classes of ecbrobots into the namespace lpzrobots
 *
 *   Revision 1.3  2008/04/08 10:11:03  guettler
 *   alpha testing
 *
 *   Revision 1.2  2008/04/08 09:09:09  martius
 *   fixed globaldata to pointer in classes
 *
 *   Revision 1.1.1.1  2008/04/08 08:14:30  guettler
 *   new ecbrobots module!
 *
 *                                                                         *
 ***************************************************************************/
#include "ecbrobot.h"

#include <selforg/matrix.h>

#include <assert.h>
#include "ecbcommunicator.h"

using namespace std;

namespace lpzrobots {


ECBRobot::ECBRobot(GlobalData& globalData) : AbstractRobot("ECBRobot", "$ID$"), globalData(&globalData) {
  if (this->globalData->debug)
    std::cout << "New ECBRobot created." << std::endl;
  // remember: motors UND sensors werden automatisch geplottet durch Agent (setMotors und getSensors)
  // add new inspectable parameters
  this->addInspectableValue("speed",&this->speed);
}

ECBRobot::~ECBRobot() {}

  /// new methods for the communicator




/// method for registering new ECBs

void ECBRobot::addECB(int slaveAddress, ECBConfig& ecbConfig) {
  this->ECBlist.push_back(new ECB(slaveAddress,*globalData,ecbConfig));
  if (globalData->debug)
    std::cout << "New ECB with address " << slaveAddress << " added." << std::endl;
  this->addInspectableValue("speedECB1",&ecb->getIrgendwas());
}


void ECBRobot::writeMotors_readSensors() {
  if (globalData->debug)
    std::cout << "ECBRobot: writeMotors_readSensors!" << std::endl;
  FOREACH(list<ECB*>,ECBlist,ecb) {
    (*ecb)->writeMotors_readSensors();
  }
}


/// ABSTRACTROBOT INTERFACE

int ECBRobot::getSensors(sensor* sensors, int sensornumber){
  int index=0;
  FOREACH(list<ECB*>,ECBlist,ecb) {
    list<sensor> ecbSensors = (*ecb)->getSensorList();
    FOREACH(list<sensor>,ecbSensors,s) {
      sensors[index++]=(*s);
      if (index==sensornumber)
        break;
    }
    if (index==sensornumber)
      break;
  }
  return sensornumber; // TODO: oder index?
}

void ECBRobot::setMotors(const motor* motors, int motornumber){
  int index=0;
  FOREACH(list<ECB*>,ECBlist,ecb) {
    index+=(*ecb)->setMotors(motors,index,motornumber);
  }
}

int ECBRobot::getSensorNumber(){
  // get the number of sensors from each ECB and sum up
  int sensornumber=0;
  FOREACH ( list<ECB*>, ECBlist, i ) {
    sensornumber+=(( *i )->getNumberSensors());
  }
  return sensornumber;
}

int ECBRobot::getMotorNumber() {
  // get the number of motors from each ECB and sum up
  int motornumber=0;
  FOREACH ( list<ECB*>, ECBlist, i ) {
    motornumber+=(( *i )->getNumberMotors());
  }
  return motornumber;
}

/** returns max number of sensors */
int ECBRobot::getMaxSensorNumber() {
  // get the number of sensors from each ECB and sum up
  int max_sensornumber=0;
  FOREACH ( list<ECB*>, ECBlist, i ) {
    max_sensornumber+=(( *i )->getMaxNumberSensors());
  }
  return max_sensornumber;
}

  /** returns maxnumber of motors */
int ECBRobot::getMaxMotorNumber() {
// get the number of motors from each ECB and sum up
  int max_motornumber=0;
  FOREACH ( list<ECB*>, ECBlist, i ) {
    max_motornumber+=(( *i )->getMaxNumberMotors());
  }
  return max_motornumber;
}

/// TRACKABLE INTERFACE

Position ECBRobot::getPosition() const {return Position(0,0,0);}
Position ECBRobot::getSpeed() const {return Position(0,0,0);}
Position ECBRobot::getAngularSpeed() const {return Position(0,0,0);}
matrix::Matrix ECBRobot::getOrientation() const { matrix::Matrix m(3,3);  m.toId(); return m; }

/// CONFIGURABLE INTERFACE

Configurable::paramval ECBRobot::getParam(const paramkey& key) const{
  /*if(key == "noise") return noise;
  else if(key == "cycletime") return cycletime;
  else if(key == "reset") return 0;
  else*/  return Configurable::getParam(key);
}

bool ECBRobot::setParam(const paramkey& key, paramval val){
  /*if(key == "noise") noise = val;
  else if(key == "cycletime"){
    cycletime=(long)val;
  } else if(key == "reset"){
    doReset=true;
  } else*/
    return Configurable::setParam(key, val);
  //return true;
}

Configurable::paramlist ECBRobot::getParamList() const {
  paramlist list;
/*  list += pair<paramkey, paramval> (string("noise"), noise);
  list += pair<paramkey, paramval> (string("cycletime"), cycletime);
  list += pair<paramkey, paramval> (string("reset"), 0);*/
  return list;
}

/**
 * Returns specific ECBRobot infos to the ECBAgent, who pipes this infos out (PlotOptions)
 * Something like that:
 * #ECB M y[0] y[1]
 * #ECB IR x[0] x[1]
 * #ECB ADC x[2] x[3]
 * #ECB ME x[4] x[5]
 * Strom, Spannung usw. (konfigurationsabhängige Parameter vom ECB)
 */
std::string ECBRobot::getGUIInformation() {
	// hole von allen ECBs die GUIInformation und konkateniere als string
	// und returne diesen als komplettstring (siehe getSensors)
	  // get the number of sensors from each ECB and sum up
	  std::string info(); // String ist Klasse
	  FOREACH ( list<ECB*>, ECBlist, i ) {
		  info.concat(( *i )->getGUIInformation()))
	  }
	  return info;
}


}
