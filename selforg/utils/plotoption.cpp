/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    joergweide84@aol.com (robot12)                                       *
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
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *                                                                         *
 *                                                                         *
 *  $Log$
 *  Revision 1.6  2009-10-27 11:44:30  robot14
 *  matrixvis added
 *
 *  Revision 1.5  2009/09/17 14:14:13  guettler
 *  added some critical qmp sections
 *
 *  Revision 1.4  2009/08/10 15:34:30  der
 *  redirect error output of neuronviz to null device
 *
 *  Revision 1.3  2009/08/05 22:53:02  martius
 *  redesigned
 *   works as a stand alone object now
 *   added init function
 *   configurables are now in engine and not in plotoptions
 *   works with wiredcontroller
 *
 *  Revision 1.2  2009/07/21 08:50:16  robot12
 *  finish of the split
 *  to do: add some comments....
 *
 *  Revision 1.1  2009/06/02 09:55:24  robot12
 *  Splitting of WiredController and PlotOption into WiredController : public PlotOptionEngine and
 *  PlotOption (used by ga_tools). Further refactorings needed.
 *                                                                                   *
 *                                                                         *
 **************************************************************************/

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
  // this prevents the simulation to terminate if the child  closes
  // or if we fail to open it.
  signal(SIGPIPE,SIG_IGN);
  switch(mode){
  case File:
      struct tm *t;
      time_t tnow;
      time(&tnow);
      t = localtime(&tnow);
      char logfilename[255];
      sprintf(logfilename,"%s_%02i-%02i-%02i_%02i-%02i-%02i.log",
	      name.c_str(), t->tm_year%100, t->tm_mon+1 , t->tm_mday,
	      t->tm_hour, t->tm_min, t->tm_sec);
      pipe=fopen(logfilename,"w");
      if (pipe)
	std::cout << "Now logging to file \"" << logfilename << "\"." << std::endl;
      break;
  case GuiLogger_File:
    pipe=popen("guilogger -m pipe -l","w");
    break;
  case GuiLogger:
    pipe=popen("guilogger -m pipe","w");
    break;
  case MatrixVis:
    pipe=popen("MatrixViz","w");
    if (pipe) std::cout << "MatrixViz-Sream opened" << std::endl;
    else std::cout << "MatrixViz-Sream open failed" << std::endl;
    break;
  case ECBRobotGUI:
    pipe=popen("SphericalRobotGUI","w");
    if (pipe)   std::cout << "open a SphericalRobotGUI-Stream " << std::endl;
    else   std::cout << "can't open SphericalRobotGUI-Stream " << std::endl;
    break;
  case NeuronViz:
    pipe=popen("neuronviz > /dev/null 2> /dev/null","w");  // TODO: Platform dependent
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
    case NeuronViz:
      //std::cout << "neuronviz pipe closing...maybe you must manually close the neuronviz first!"
      //          << std::endl;
      // send quit message to pipe
      fprintf(pipe, "#QUIT\n");
      pclose(pipe);
      std::cout << "neuronviz pipe closing...SUCCESSFUL" << std::endl;
      break;
    case MatrixVis:
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
    case NeuronViz:
    case MatrixVis:
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
