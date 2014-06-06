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
#include <assert.h>

#include "simulation.h"

#include "replayrobot.h"
using namespace std;
using namespace matrix;

namespace lpzrobots {

  ReplayRobot::ReplayRobot(const OdeHandle& odeHandle,
                 const OsgHandle& osgHandle,
                 const char* filename)
    : OdeRobot(odeHandle, osgHandle, "ReplayRobot", "$Id$"),
      filename(filename){

    f=fopen(filename,"r");
    if(!f){
      cerr<< "ReplayRobot: error while opening file " << filename << endl;
      exit(1);
    }
    if(!parseDataFileForHeader(f, sensorStart, sensorEnd,  motorStart, motorEnd)){
      cerr<< "ReplayRobot: error while seaching for header in file " << filename << endl;
      exit(1);
    }
    printf("Test: %i, %i, %i, %i\n", sensorStart, sensorEnd, motorStart, motorEnd);

  };

  ReplayRobot::~ReplayRobot(){
    if(f) fclose(f);
  }

  void ReplayRobot::setMotorsIntern(const double* _motors, int motornumber){
  };

  int ReplayRobot::getSensorsIntern(sensor* s, int sensornumber){
    assert(sensornumber == (sensorEnd-sensorStart + 1));
    if(!parseDataLine(sensors,f)){
      cout << "ReplayRobot: no datafile in file" << endl;
    }else{
      sensors=sensors.rows(sensorStart, sensorEnd);
    }
    sensors.convertToBuffer(s,sensorEnd-sensorStart + 1);
    return sensorEnd-sensorStart + 1;
  };

  bool ReplayRobot::parseDataFileForHeader(FILE* f, int & sensorstart, int& sensorend,  int& motorstart, int& motorend){
    char buffer[1024];
    int i;
    sensorstart=-1;
    sensorend=-1;
    motorstart=-1;
    motorend=-1;

    while(fgets(buffer, 1024, f)) {
      if(buffer[0]=='#' && buffer[1]=='C'){
        // scan line and return
        i=0;
        char* p;
        p=strtok(buffer," ");
        if(!p) return false; // frist one is #C
        while((p=strtok(NULL," "))!=NULL )  {
          if(p[0]=='x' && p[1]=='['){
            if(sensorstart==-1) sensorstart=i;
            sensorend=i;
          }
          if(p[0]=='y' && p[1]=='['){
            if(motorstart==-1) motorstart=i;
            motorend=i;
          }
          i++;
        }
        return true;
      }
    }
    return false;
  }

  bool ReplayRobot::isEmpty(const char* c){
    const char* p = c;
    bool foundsomething = false;
    while(*p != 0){
      if(*p > ' ') foundsomething = true;
      p++;
    }
    return !foundsomething;
  }


  bool ReplayRobot::check4Number(const char* c){
    const char* p = c;
    while(*p != 0){
      if(*p >= '0' && *p <= '9') return true;
      p++;
    }
    return false;
  }

  bool ReplayRobot::parseDataLine(Matrix& data, FILE* f){
    char buffer[1024];
    int i;
    double dat[1024];
    while(fgets(buffer, 1024, f)){
      if(buffer[0]=='#' || isEmpty(buffer)){
        continue;
      }else{
        i=0;
        char* p;
        p=strtok(buffer," ");
        if(!p) return false;
        dat[i] = atof(p);
        i++;
        while((p=strtok(NULL," "))!=NULL )  {
          if(!check4Number(p)) continue;
          dat[i] = atof(p);
          i++;
        };
        data.set(i,1,dat);
        return true;
      }
    };
    return false;
  }

}
