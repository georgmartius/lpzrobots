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

#include "plotoption.h"
#include <iostream>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <quickmp.h>

bool PlotOption::open(){
  QMP_CRITICAL(601);
  char cmd[255];
  bool returnCode = true;
  std::cout << "open a stream " << std::endl;
  switch(mode){
  case File:
      struct tm *t;
      time_t tnow;
      time(&tnow);
      t = localtime(&tnow);
      char logfilename[255];
      if (parameter=="no_time_in_filename"){
        sprintf(logfilename,"%s.log",name.c_str());
      } else{
	sprintf(logfilename,"%s_%02i-%02i-%02i_%02i-%02i-%02i.log",
	      name.c_str(), t->tm_year%100, t->tm_mon+1 , t->tm_mday,
  	      t->tm_hour, t->tm_min, t->tm_sec);
      }
      pipe=fopen(logfilename,"w");
      if (pipe)
	std::cout << "Now logging to file \"" << logfilename << "\"." << std::endl;
      break;
  case GuiLogger_File:
    pipe=popen("guilogger -m pipe -l","w");
    break;
  case GuiLogger:
    sprintf(cmd, "guilogger -m pipe %s", parameter.c_str());
    pipe=popen(cmd,"w");
    break;
  case MatrixViz:
    pipe=popen("matrixviz -noCtrlC","w");
    if (pipe) std::cout << "MatrixViz-Sream opened" << std::endl;
    else std::cout << "MatrixViz-Sream open failed" << std::endl;
    break;
  case ECBRobotGUI:
    pipe=popen("SphericalRobotGUI","w");
    if (pipe)   std::cout << "open a SphericalRobotGUI-Stream " << std::endl;
    else   std::cout << "can't open SphericalRobotGUI-Stream " << std::endl;
    break;
  case SoundMan:
    sprintf(cmd,"soundMan %s",parameter.c_str());
    pipe=popen(cmd,"w");
    break;
  default: // and NoPlot
    returnCode=false;
  }
  if(pipe==0){
    fprintf(stderr, "%s:%i: could not open plot tool!\n", __FILE__, __LINE__);
    returnCode=false;
  }
  QMP_END_CRITICAL(601);
  return returnCode;
}


void PlotOption::close(){
  QMP_CRITICAL(602);
  if (pipe) {

    switch(mode){
    case File:
      std::cout << "logfile closing...SUCCESSFUL" << std::endl;
      fclose(pipe);
      break;
    case GuiLogger:
    case GuiLogger_File:
      //std::cout << "guilogger pipe closing...maybe you must manually close the guilogger first!"
      //          << std::endl;
      // send quit message to pipe
      fprintf(pipe, "#QUIT\n");
      fflush(pipe);
      pclose(pipe);
      std::cout << "guilogger pipe closing...SUCCESSFUL" << std::endl;
      break;
    case MatrixViz:
    //       std::cout << "Try to close ECBRobotGUI pipe...";
      fprintf(pipe, "#QUIT\n");
      pclose(pipe);
      std::cout << "MatixViz pipe closing...SUCCESSFUL" << std::endl;
      break;
    case ECBRobotGUI:
//       std::cout << "Try to close ECBRobotGUI pipe...";
      fprintf(pipe, "#QUIT\n");
      pclose(pipe);
      std::cout << "ECBRobotGUI pipe closing...SUCCESSFUL" << std::endl;
      break;
    case SoundMan:
      std::cout << "SoundMan closing...SUCCESSFUL" << std::endl;
      fclose(pipe);
      break;

    default:
      break;
    }
    pipe=0;
  }
  QMP_END_CRITICAL(602);
}

// flushes pipe (depending on mode)
void PlotOption::flush(long step){
  QMP_CRITICAL(603);
  if (pipe) {
    switch(mode){
    case File:
      if((step % (interval * 1000)) == 0) fflush(pipe);
      break;
    case GuiLogger:
    case GuiLogger_File:
    case MatrixViz:
    case ECBRobotGUI:
    case SoundMan:{
      int ttt = fflush(pipe);
      if(ttt!=0) {
	printf("Pipe broken: %s\n",strerror(ttt));
	close();
      }
      break;}
    default:
      break;
    }
  }
  QMP_END_CRITICAL(603);
}
