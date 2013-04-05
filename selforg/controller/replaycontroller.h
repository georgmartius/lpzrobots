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
#ifndef __REPLAYCONTROLLER_H
#define __REPLAYCONTROLLER_H

#include "abstractcontroller.h"

/**
 * Controller that replays a file
 */
class ReplayController : public AbstractController {
public:
  ReplayController(const char* filename, bool repeat=false)
    : AbstractController("ReplayController", "1.0"),
      filename(filename), repeat(repeat) {

    f=fopen(filename,"r");
    if(!f){
      std::cerr<< "ReplayController: error while opening file " << filename << std::endl;
      exit(1);
    }
    if(!parseDataFileForHeader(f)){
      std::cerr<< "ReplayController: error while seaching for header in file " << filename << std::endl;
      exit(1);
    }
    printf("ReplayController: columns: Senors [%i, %i], Motors [%i, %i]\n",
           sensorStart, sensorEnd, motorStart, motorEnd);
  }

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0){
    //assert(sensornumber == sensorEnd - sensorStart + 1);
    assert(motornumber  == motorEnd - motorStart + 1);
  }

  virtual int getSensorNumber() const { return sensorEnd - sensorStart + 1;};

  virtual int getMotorNumber() const { return motorEnd - motorStart + 1; };

  virtual void step(const sensor* sensors, int sensornumber,
                    motor* motors, int motornumber){
    stepNoLearning(sensors,sensornumber, motors, motornumber);
  }

  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* motors, int number_motors){

    if(!parseDataLine(m,f)){
      if(repeat){
        std::cout << "ReplayController: rewind" << std::endl;
        rewind(f);
      }else
        std::cout << "ReplayController: no datafile in file" << filename << std::endl;
    }else{
      m=m.rows(motorStart, motorEnd);
    }
    m.convertToBuffer(motors, motorEnd-motorStart + 1);
  }

  /**** STOREABLE ****/
  /** stores the controller values to a given file (binary).  */
  virtual bool store(FILE* f) const {return false;}
  /** loads the controller values from a given file (binary). */
  virtual bool restore(FILE* f) {return false;}

  // inspectable interface
  virtual std::list<iparamkey> getInternalParamNames()const  { return std::list<iparamkey>(); }
  virtual std::list<iparamval> getInternalParams() const { return std::list<iparamval>(); }

protected:

  bool parseDataFileForHeader(FILE* f){
    char buffer[1024];
    int i;
    sensorStart=-1;
    sensorEnd=-1;
    motorStart=-1;
    motorEnd=-1;

    while(fgets(buffer, 1024, f)) {
      if(buffer[0]=='#' && buffer[1]=='C'){
        // scan line and return
        i=0;
        char* p;
        p=strtok(buffer," ");
        if(!p) return false; // frist one is #C
        while((p=strtok(NULL," "))!=NULL )  {
          if(p[0]=='x' && p[1]=='['){
            if(sensorStart==-1) sensorStart=i;
            sensorEnd=i;
          }
          if(p[0]=='y' && p[1]=='['){
            if(motorStart==-1) motorStart=i;
            motorEnd=i;
          }
          i++;
        }
        return true;
      }
    }
    return false;
  }

  static bool isEmpty(const char* c){
    const char* p = c;
    bool foundsomething = false;
    while(*p != 0){
      if(*p > ' ') foundsomething = true;
      p++;
    }
    return !foundsomething;
  }


  static bool check4Number(const char* c){
    const char* p = c;
    while(*p != 0){
      if(*p >= '0' && *p <= '9') return true;
      p++;
    }
    return false;
  }

  static bool parseDataLine(matrix::Matrix& data, FILE* f){
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



protected:
  int sensorStart;
  int sensorEnd;
  int motorStart;
  int motorEnd;
  matrix::Matrix m;
  const char* filename;
  FILE* f;
  bool repeat;

};

#endif
