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
 *   Revision 1.6  2008-08-12 11:45:00  guettler
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
#include <selforg/storeable.h>
#include <selforg/types.h>

#include "globaldata.h"

#include <list>


namespace lpzrobots {

// forward declaration begin
class ECBCommunicator;
// forward declaration end

typedef enum {
	ADC_TILT,

	ADC_IR,

	ADC_COMPASS,

	ADC_DEFAULT
} ADCType;

typedef struct {
  /// set I2C on or off
  bool useI2C;
  /// set ADC on or off
  bool useADC;
  /// enable ADC plug and play (guessing)
  bool useJumperedADC_PlugNPlay;
  /// set SPI on or off
  bool useSPI;

  /// the max number, important for init of controller
  int maxNumberMotors;
  /// the max number, important for init of controller
  int maxNumberSensors;

  /// define here which adc-sensors are active.
  /// 00100000 -> adc[5] = adc-sensor 5 is online
  int ADCSensorMask;

  ADCType adcTypes[8];

} ECBConfig;


class ECB : public Configurable, public Storeable {
public:

  /**
   * Creates the ECB with the given address. Note that the configuration
   * is read from the configuration file "ecb{address}.cfg"!
   * @param address
   */
  ECB(short address, GlobalData& globalData,ECBConfig& ecbConfig);

  virtual ~ECB();

  // TODO: needed????
//   virtual bool isInitialised() const { return initialised; }
//
//   virtual void setInitialised(bool initialised) { this->initialised=initialised; }

  virtual short getAddress() const {	return address; }

  virtual std::list<sensor> getSensorList() { return sensorList; }

  virtual std::list<motor> getMotorList() { return motorList; }

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
  virtual int setMotors(const motor* motorArray,int beginIndex,int maxIndex);

  /// CONFIG VARS
  static ECBConfig getDefaultConf(){
    ECBConfig conf;
    conf.useI2C = true;
    conf.useADC = true;
    conf.useJumperedADC_PlugNPlay = true;
    conf.useSPI=false;
    conf.maxNumberMotors = 4;
    conf.maxNumberSensors = 20;
    conf.ADCSensorMask = 00000000;
    return conf;
  }


  /**
   * Sends the new motor values to the ECB and receives the
   * new sensor values
   * @return true if the communication was successful
   */
  virtual bool writeMotors_readSensors();

  /**
  * Send reset command to the ECB and receive
  * the number of connected motors and sensors
  * If the reset fails, the ECB stays in uninitialized mode!
  * @return
  */
  virtual bool resetECB();


  /**
   * Sends the beep command to the ECB
   * @return true if send of the command was successful
   */
  virtual bool makeBeepECB();

  /// STORABLE INTERFACE

  /** stores the object to the given file stream (binary). */
  virtual bool store(FILE* f) const;

  /** loads the object from the given file stream (binary). */
  virtual bool restore(FILE* f);


private:
  GlobalData* globalData;
  ECBConfig ecbConfig;

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

  // die Adresse, die das ECB hat, d.h. muss im ECB hardprogrammiert sein
  short address;

  // siehe initialised, wenn failurecounter bestimmten wert überschritten,
  // dann reset im nächsten step versuchen
  int failureCounter;

  /// noch verschiedene sensortypen und motortypen berücksichtigen
  /// bzw. hier bzw. in cpp-datei die cfg-datei berücksichtigen
  /// Informationen vorhalten (bekommt man bei Antwort auf SW-RESET),
  /// welche ADC, IRS, Motorentypen usw. wie zu den Sensoren und Motoren
  /// (siehe Listen oben) zugeordnet sind (k�me auch in die ECBConfig rein,
  /// insoweit von der main.cpp aus konfigurierbar (nicht plug&playf�hige Sachen))

  /// diese Infos an ECBRobot weiterleiten (die holt er sich):
  /**
   * Returns specific ECBRobot infos to the ECBAgent, who pipes this infos out (PlotOptions)
   * Something like that:
   * #ECB M y[0] y[1]
   * #ECB IR x[0] x[1]
   * #ECB ADC x[2] x[3]
   * #ECB ME x[4] x[5]
   * Strom, Spannung usw. (konfigurationsabh�ngige Parameter vom ECB)
   */
  virtual std::string getGUIInformation();



  /**
   * Converts a given byteVal to a double value
   * byteVal=0   -> doubleVal=-1
   * byteVal=127 -> doubleVal=0
   * byteVal=255 -> doubleVal=1
   * @param byteVal
   * @return
   */
  virtual double convertToDouble(int byteVal);

  /**
   * Converts a given doubleVal to a byte value
   * doubleVal=-1 -> byteVal=0
   * doubleVal=0  -> byteVal=127
   * doubleVal=1  -> byteVal=255
   * @param doubleVal
   * @return
   */
  virtual int convertToByte(double doubleVal);

};

}

#endif
