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
 *   Revision 1.12  2011-03-22 16:37:18  guettler
 *   - adpaptions to enhanced configurable and inspectable interface
 *
 *   Revision 1.11  2011/03/21 17:31:30  guettler
 *   - adapted to enhance Inspectable interface (has now a name shown also in GuiLogger)
 *
 *   Revision 1.10  2010/11/11 15:34:59  wrabe
 *   - some extensions for QMessageClient (e.g. quitServer())
 *   - fixed some includes
 *
 *   Revision 1.9  2010/11/10 09:32:00  guettler
 *   - port to Qt part 1
 *
 *   Revision 1.8  2010/04/28 08:09:30  guettler
 *   - bugfixes
 *   - stopMotors is now invoked by ECBCommunicator
 *
 *   Revision 1.7  2009/08/11 15:49:05  guettler
 *   Current development state:
 *   - Support of communication protocols for XBee Series 1, XBee Series 2 and cable mode
 *   - merged code base from ecb_robots and Wolgang Rabes communication handling;
 *     ECBCommunicator (almost) entirely rewritten: Use of Mediator (MediatorCollegues: ECB),
 *     Callbackble (BackCaller: SerialPortThread)
 *   - New CThread for easy dealing with threads (is using pthreads)
 *   - New TimerThreads for timed event handling
 *   - SerialPortThread now replaces the cserialthread
 *   - QGlobalData, ECBCommunicator is now configurable
 *   - ECBAgent rewritten: new PlotOptionEngine support, adapted to new WiredController structure
 *   - ECBRobot is now Inspectables (uses new infoLines functionality)
 *   - ECB now supports dnsNames and new communication protocol via Mediator
 *   - Better describing command definitions
 *   - SphericalRobotECB: adapted to new ECB structure, to be tested
 *
 *   Revision 1.6  2009/03/25 11:06:55  robot1
 *   updated version
 *
 *   Revision 1.5  2008/08/12 11:44:42  guettler
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
#include "QECBCommunicator.h"
#include <string>

using namespace std;

namespace lpzrobots {

  ECBRobot::ECBRobot(QGlobalData& globalData) :
    AbstractRobot("ECBRobot", "$ID$"), Inspectable("ECBRobot"), globalData(globalData), numberECBInitialised(0), initialised(false) {
    globalData.textLog("New ECBRobot created.");

    // remember: motors UND sensors werden automatisch geplottet durch Agent (setMotors und getSensors)
    // add new inspectable parameters
    // but we must add the descriptionLines
    addInspectableValue("numberECBs", &numberECBInitialised);

    addParameterDef("reset", &isResetECBs, false);
  }

  ECBRobot::~ECBRobot() {
    FOREACH(list<ECB*>,ECBlist,ecb) {
      delete (*ecb);
    }
  }

  /// new methods for the communicator


  /// method for registering new ECBs

  void ECBRobot::addECB(QString dnsName, ECBConfig& ecbConfig) {
    ECB* ecb = new ECB(dnsName, globalData, ecbConfig);
    this->ECBlist.push_back(ecb);
    addConfigurable(ecb);
    addInspectable(ecb);
    globalData.textLog("New ECB with DNSName \"" +dnsName + "\" added.");
  }

  /// method for registering new ECBs

  void ECBRobot::addECB(ECB* ecb) {
    this->ECBlist.push_back(ecb);
    addConfigurable(ecb);
    addInspectable(ecb);
    globalData.textLog("New ECB with DNSName " + ecb->getDNSName() + " added.");
  }

  bool ECBRobot::isInitialised() {
    if (!initialised) {
      // sum up number of initialised ECBs
      int count = 0;

      FOREACH(list<ECB*>,ECBlist,ecb) {
        if ((*ecb)->isInitialised())
          count++;
      }
      numberECBInitialised = count;
      if (numberECBInitialised == ECBlist.size() && getMotorNumber()>0 && getSensorNumber() >0) {
        globalData.textLog("ECBRobot: Found all (" + QString::number(numberECBInitialised) + ") initialised ECBs.");
        removeInfoLines();
        addInfoLine(getChannelDescription());
        initialised = true;
        return true;
      } else
        return false;
    } else if (isResetECBs == true) {
      resetECBs();
      initialised = false;
      isResetECBs = false;
      return false;
    } else // initialised && ! resetECBs
      return true;
  }
  
  void ECBRobot::resetECBs() {
    globalData.textLog("ECBRobot: resetECBs!");
    FOREACH(list<ECB*>,ECBlist,ecb) {
      (*ecb)->sendResetECB();
    }
  }
  
  /// ABSTRACTROBOT INTERFACE

  int ECBRobot::getSensors(sensor* sensors, int sensornumber) {
    int index = 0;
    FOREACH(list<ECB*>,ECBlist,ecb) {
      list<sensor> ecbSensors = (*ecb)->getSensorList();
      FOREACH(list<sensor>,ecbSensors,s) {
        sensors[index++] = (*s);
        if (index == sensornumber)
          break;
      }
      if (index == sensornumber)
        break;
    }
    return sensornumber; // TODO: oder index?
  }

  void ECBRobot::setMotors(const motor* motors, int motornumber) {
    int index = 0;
    FOREACH(list<ECB*>,ECBlist,ecb) {
      index += (*ecb)->setMotors(motors, index, motornumber);
    }
  }

  int ECBRobot::getSensorNumber() {
    // get the number of sensors from each ECB and sum up
    int sensornumber = 0;
    FOREACH ( list<ECB*>, ECBlist, i ) {
      sensornumber += ((*i)->getNumberSensors());
    }
    return sensornumber;
  }

  int ECBRobot::getMotorNumber() {
    // get the number of motors from each ECB and sum up
    int motornumber = 0;
    FOREACH ( list<ECB*>, ECBlist, i ) {
      motornumber += ((*i)->getNumberMotors());
    }
    return motornumber;
  }

  /** returns max number of sensors */
  int ECBRobot::getMaxSensorNumber() {
    // get the number of sensors from each ECB and sum up
    int max_sensornumber = 0;
    FOREACH ( list<ECB*>, ECBlist, i ) {
      max_sensornumber += ((*i)->getMaxNumberSensors());
    }
    return max_sensornumber;
  }

  /** returns maxnumber of motors */
  int ECBRobot::getMaxMotorNumber() {
    // get the number of motors from each ECB and sum up
    int max_motornumber = 0;
    FOREACH ( list<ECB*>, ECBlist, i ) {
      max_motornumber += ((*i)->getMaxNumberMotors());
    }
    return max_motornumber;
  }

  /** stop all motors of connected ECBs */
  int ECBRobot::stopMotors() {
    globalData.textLog("ECBRobot: stopMotors()");
    FOREACH ( list<ECB*>, ECBlist, i ) {
      (*i)->stopMotors();
    }
    return 0;
  }

  /** start all motors of connected ECBs */
  int ECBRobot::startMotors() {
    globalData.textLog("ECBRobot: startMotors()");
    FOREACH ( list<ECB*>, ECBlist, i ) {
      (*i)->startMotors();
    }
    return 0;
  }

  /// TRACKABLE INTERFACE

  Position ECBRobot::getPosition() const {
    return Position(0, 0, 0);
  }
  Position ECBRobot::getSpeed() const {
    return Position(0, 0, 0);
  }
  Position ECBRobot::getAngularSpeed() const {
    return Position(0, 0, 0);
  }
  matrix::Matrix ECBRobot::getOrientation() const {
    matrix::Matrix m(3, 3);
    m.toId();
    return m;
  }

  /**
   * Returns specific ECBRobot infos to the ECBAgent, who pipes this infos out (PlotOptions)
   * Something like that:
   * #ECB M y[0] y[1]
   * #ECB IR x[0] x[1]
   * #ECB ADC x[2] x[3]
   * #ECB ME x[4] x[5]
   * Strom, Spannung usw. (konfigurationsabh�ngige Parameter vom ECB)
   */
  std::string ECBRobot::getChannelDescription() {
    // hole von allen ECBs die GUIInformation und konkateniere als string
    // und returne diesen als komplettstring (siehe getSensors)
    // get the number of sensors from each ECB and sum up
    std::string info = "D"; // String ist Klasse
    FOREACH ( list<ECB*>, ECBlist, i ) {
      info.push_back(' ');
      info.append((*i)->getChannelDescription());
    }
    return info;
  }

}

