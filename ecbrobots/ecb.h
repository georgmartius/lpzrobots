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
 *   Revision 1.14  2011-03-22 16:36:29  guettler
 *   - ECB is now inspectable
 *
 *   Revision 1.13  2011/02/04 12:59:05  wrabe
 *   - convert function for short values added
 *
 *   Revision 1.12  2010/11/11 15:34:59  wrabe
 *   - some extensions for QMessageClient (e.g. quitServer())
 *   - fixed some includes
 *
 *   Revision 1.11  2010/11/10 09:32:00  guettler
 *   - port to Qt part 1
 *
 *   Revision 1.10  2009/08/18 14:49:37  guettler
 *   implemented COMMAND_MOTOR_MAX_CURRENT
 *
 *   Revision 1.9  2009/08/11 15:49:05  guettler
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
 *   Revision 1.8  2009/03/25 11:06:55  robot1
 *   updated version
 *
 *   Revision 1.7  2008/08/15 13:16:58  robot1
 *   add PORT PIN configuration for ECBs
 *
 *   Revision 1.6  2008/08/12 11:45:00  guettler
 *   plug and play update, added some features for the ECBRobotGUI
 *
 *   Revision 1.5  2008/07/16 07:38:42  robot1
 *   some major improvements
 *
 *   Revision 1.4  2008/04/11 06:31:16  guettler
 *   Included all classes of ecbrobots into the namespace lpzrobots
 *
 *   Revision 1.3  2008/04/11 06:15:48  guettler
 *   Inserted convertion from byte to double and backwards for motor and sensor values
 *
 *   Revision 1.2  2008/04/08 09:09:09  martius
 *   fixed globaldata to pointer in classes
 *
 *   Revision 1.1.1.1  2008/04/08 08:14:30  guettler
 *   new ecbrobots module!
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __ECB_H
#define __ECB_H

#include <selforg/configurable.h>
#include <selforg/inspectable.h>
#include <selforg/storeable.h>
#include <selforg/types.h>
#include <selforg/mediatorcollegue.h>

#include "types.h"

#include <list>
#include <string>

#include <QString>

// #define PORTA ecbPort[0]
// #define PORTB ecbPort[1]
// #define PORTC ecbPort[2]
// #define PORTD ecbPort[3]
// #define PORTE ecbPort[4]
// #define PORTF ecbPort[5]
// #define PORTG ecbPort[6]

namespace lpzrobots
{

  // forward declaration begin
  class ECBCommunicator;
  class ECBCommunicationEvent;
  struct QGlobalData;
  // forward declaration end


  typedef struct
  {

      /// the max number, important for init of controller
      int maxNumberMotors;
      /// the max number, important for init of controller
      int maxNumberSensors;
      /// Sets the maximum current for the motorboards (from Wolfgang Rabe, Version 1.0).
      uint8 maxMotorCurrent;

  } ECBConfig;

  class ECB : public Configurable, public Inspectable, public Storeable, public MediatorCollegue
  {
    public:

      static const uint8 DEFAULT_MAX_MOTOR_CURRENT = 80;

      /**
       * Creates the ECB with the given address. Note that the configuration
       * is read from the configuration file "ecb{address}.cfg"! (in future releases)
       * @param dnsName the name of the ECB. This name is used to identify the ECB in
       * over wireless connection (XBee). The XBee uses the same dnsName.
       * @param globalData holds all necessary global data.
       * @param ecbConfig the configuration for this ECB, e.g. maximum number of motors
       * @return
       */
      ECB(QString dnsName, QGlobalData& globalData, ECBConfig& ecbConfig);

      virtual ~ECB();

      virtual QString getDNSName() const
      {
        return dnsName;
      }

      virtual std::list<sensor> getSensorList()
      {
        return sensorList;
      }

      virtual std::list<motor> getMotorList()
      {
        return motorList;
      }

      virtual int getNumberMotors();

      virtual int getNumberSensors();

      virtual int getMaxNumberMotors();

      virtual int getMaxNumberSensors();

      /**
       * Is called from the ECBRobot.
       * @param motorArray the array that contains the new motor values
       * @param beginIndex the index where the proper values start
       * @param maxIndex  the maximum index which is allowed for motorArray
       * @return The number of motor values read (used by the ECB)
       */
      virtual int setMotors(const motor* motorArray, int beginIndex, int maxIndex);

      /// CONFIG VARS
      static ECBConfig getDefaultConf()
      {
        ECBConfig conf;
        conf.maxNumberMotors = 2;
        conf.maxNumberSensors = 16;
        conf.maxMotorCurrent = DEFAULT_MAX_MOTOR_CURRENT;
        return conf;
      }

      /**
       * Send reset command to the ECB and receive
       * the number of connected motors and sensors
       * If the reset fails, the ECB stays in uninitialized mode!
       */
      virtual void sendResetECB();

      virtual void sendMotorValuesPackage();

      /**
       * Send stop command to the ECB to disable the motors
       */
      virtual void stopMotors();

      /**
       * Send start command to the ECB to enable the motors
       */
      virtual void startMotors();

      /**
       * Sets the maximum current for the motorboards (from Wolfgang Rabe, Version 1.0).
       * If the current is exceeded, the board limits the current to the maximum
       * (the pulse length of the pwm-signal for the motor driver stage is shortened).
       * valid interval is [60;250]. Default value: 80
       * This limits the current between 600mA and 2500mA.
       * Remember, that the nominal current is 500mA (600mA are secured by motorboard).
       * High currents stresses the motors and motorcontroller!
       * The max recommended current is 1500mA.
       */
      virtual void sendMaxMotorCurrent(uint8 maxCurrent, uint8 motorboardIndex=0);

      /**
       * Sends the beep command to the ECB
       */
      virtual void makeBeepECB();

      /// STORABLE INTERFACE

      /** stores the object to the given file stream (binary). */
      virtual bool store(FILE* f) const;

      /** loads the object from the given file stream (binary). */
      virtual bool restore(FILE* f);

      /// noch verschiedene sensortypen und motortypen berücksichtigen
      /// bzw. hier bzw. in cpp-datei die cfg-datei berücksichtigen
      /// Informationen vorhalten (bekommt man bei Antwort auf SW-RESET),
      /// welche ADC, IRS, Motorentypen usw. wie zu den Sensoren und Motoren
      /// (siehe Listen oben) zugeordnet sind (käme auch in die ECBConfig rein,
      /// insoweit von der main.cpp aus konfigurierbar (nicht plug&playfähige Sachen))

      /// diese Infos an ECBRobot weiterleiten (die holt er sich):
      /**
       * Returns specific ECBRobot infos to the ECBAgent, who pipes this infos out (PlotOptions)
       * Something like that:
       * #ECB M y[0] y[1]
       * #ECB IR x[0] x[1]
       * #ECB ADC x[2] x[3]
       * #ECB ME x[4] x[5]
       * Strom, Spannung usw. (konfigurationsabhängige Parameter vom ECB)
       */
      virtual std::string getChannelDescription();

      virtual bool isInitialised()
      {
        return this->initialised;
      }

      /**
       * Is called when the mediator informs this collegue that an event
       * has to be performed by this collegue instance.
       */
      virtual void doOnMediatorCallBack(MediatorEvent* event);

      virtual void commandDimensionReceived(ECBCommunicationEvent* event);

      virtual void commandSensorsReceived(ECBCommunicationEvent* event);


    protected:
      QGlobalData* globalData;
      ECBConfig ecbConfig;

      std::string descriptionLine;

      int currentNumberSensors;
      int currentNumberMotors;

      std::list<sensor> sensorList;
      std::list<motor> motorList;

      std::string infoString;

      // gibt an, ob Reset-Command bereits erfolgreich
      // es kann sein, dass ECB mehrmals nicht antwortet, dann setze
      // initialise=0 und versuche im nächsten Step, neu zu resetten,
      // Wenn reset nicht erfolgreich, übergehe zeitweilig ECB (wegen Funkstörung)
      bool initialised;

      /// this is the node identifier, hardware programmed on ECB (EEPROM)
      QString dnsName;

      // siehe initialised, wenn failurecounter bestimmten wert überschritten,
      // dann reset im nächsten step versuchen
      int failureCounter;

      /**
       * Converts a given byteVal to a double value
       * byteVal=0   -> doubleVal=-1
       * byteVal=127 -> doubleVal=0
       * byteVal=255 -> doubleVal=1
       * @param byteVal
       * @return
       */
      virtual double convertToDouble(uint8 byteVal);

      /**
       * Converts a given doubleVal to a byte value
       * doubleVal=-1 -> byteVal=0
       * doubleVal=0  -> byteVal=127
       * doubleVal=1  -> byteVal=255
       * @param doubleVal
       * @return
       */
      virtual uint8 convertToByte(double doubleVal);


      /**
       * Converts a given shortVal to a double value
       * shortVal=minBound   -> doubleVal=-1
       * shortVal=(minBound + maxBound) / 2 -> doubleVal=0 (zero point is shifted with average of minBound and maxBound)
       * shortVal=maxBound   -> doubleVal=1
       * @param shortVal
       * @param minBound the value which indicates the minimal operating range (see above)
       * @param maxBound the value which indicates the maximal operating range (see above)
       * @return
       */
      virtual double convertToDouble(short shortVal, short minBound, short maxBound);

      /**
       * Converts a given doubleVal to a unsigned short value
       * doubleVal=-1 -> shortVal=minBound
       * doubleVal=0  -> shortVal=(minBound + maxBound) / 2
       * doubleVal=1  -> shortVal=maxBound
       * @param doubleVal
       * @param minBound the value which indicates the minimal operating range (see above)
       * @param maxBound the value which indicates the maximal operating range (see above)
       * @return
       */
      virtual short convertToShort(double doubleVal, short minBound, short maxBound);


      /**
       * Useful for command packages which contain no additional
       * data, so that dataLength=0.
       * @param command the command to send to the ECB
       */
      void generateAndFireEventForCommand(uint8 command);

  };

}

#endif
