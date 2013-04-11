/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
#ifndef __USE_JAVA_CONTROLLER_H
#define __USE_JAVA_CONTROLLER_H

#ifndef WIN32

//#include <stdio.h>
#include "abstractcontroller.h"

//server
//#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <exception>


#define BUFFER_SIZE 1024
#define MAX_CONFIG_PARAM 50
#define MAX_INTERNAL_PARAM 50


double atof ( const char * );
int    ctoi ( const char * );


using namespace std;

/**
 * class for robot control with sine and cosine
 *
 *
 */
class use_java_controller : public AbstractController
{






        public:

                static int anzahl_Java_controller;


                use_java_controller ( const char* port_controller = "4444", const char* port_internalParams = NULL, const char* name = "defaultRobot" );


                static void addController(){use_java_controller::anzahl_Java_controller++;}


                virtual ~use_java_controller();

                /** initialisation of the controller with the given sensor/ motornumber
                    Must be called before use.
                */
  virtual void init ( int sensornumber, int motornumber, RandGen* randgen = 0);

                /** @return Number of sensors the controller was initialised
                    with or 0 if not initialised */
                virtual int getSensorNumber() const {return number_sensors;}


                /** @return Number of motors the controller was initialised
                    with or 0 if not initialised */
                virtual int getMotorNumber() const {return number_motors;}

                /** performs one step (includes learning).
                    Calculates motor commands from sensor inputs.
                    @param sensors sensors inputs scaled to [-1,1]
                    @param sensornumber length of the sensor array
                    @param motors motors outputs. MUST have enough space for motor values!
                    @param motornumber length of the provided motor array
                */
                virtual void step ( const sensor* sensors, int sensornumber,
                                    motor* motors, int motornumber );
                /** performs one step without learning.
                    @see step
                */
                virtual void stepNoLearning ( const sensor* , int number_sensors,
                                              motor* , int number_motors );

                /**
                  * Methode verschickt message an Java-controller
                  */
                void sendToJava ( const char* message,bool abbruch, const char* meldung="Fehler beim Senden der Daten zum Java-Controller\n" );

        void closeJavaController();


                /**
                * executed once when guilogger or neuronvis or file logger is started to get the names of the
                inspectable parameters (names should be sent from java-controller and returned)
                */
                virtual std::list<iparamkey> getInternalParamNames() const;
                /**
                * executed every step when guilogger or neuronvis or file logger is active to get the values of the
                inspectable parameters (values should be sent from java-controller and returned)
                */

                virtual std::list<iparamval> getInternalParams() const;

                // next three described in cpp-file
                virtual paramval getParam ( const paramkey& key ) const;
                virtual bool setParam ( const paramkey& key, paramval val );
                virtual paramlist getParamList() const ;



                virtual bool store   ( FILE* f ) const { return true;};  // FIXME: store Parameter
                virtual bool restore ( FILE* f )       { return true;};  // FIXME: restore Parameter


        protected:

                int t;
                const char* name;
                int number_sensors;
                int number_motors;
                int number_controlled;

                bool serverOK;

                int server_controller;
                int server_internalParams;
                int client_controller;
                int client_internalParams;

                socklen_t client_controller_size;
                socklen_t client_internalParams_size;

                struct sockaddr_in server_controller_addr;
                struct sockaddr_in server_internalParams_addr;
                struct sockaddr_in client_controller_addr;
                struct sockaddr_in client_internalParams_addr;


                paramlist config_param_list;
                int anz_config_param;

                iparamkeylist internal_keylist;
                iparamvallist internal_vallist;
                iparamvallist internal_vallist_alt;
                int anz_internal_param;

                bool can_send;
            bool isFirst;
                bool isClosed;
                bool server_guilogger_isClosed;
                bool server_controller_isClosed;

                int anzahl_closed_Server;

                double *motor_values_alt;

};

#endif // win32

#endif
