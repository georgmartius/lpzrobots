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
#include <sstream>
#include <signal.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <quickmp.h>
#include <selforg/stl_adds.h>
#include <selforg/inspectable.h>

using namespace std;

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
      if (!parameter.empty()){
        sprintf(logfilename,"%s%s.log", parameter.c_str(), name.c_str());
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
    pipe=popen("matrixviz -noCtrlC -novideo","w");
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

void PlotOption::setFilter(const list<string>& accept, const list<string>& ignore){
  this->accept = accept;
  this->ignore = ignore;
}

/// sets a filter to this plotoption: syntax: +acceptregexp -ignoreregexp ...
void PlotOption::setFilter(const std::string filter){
  //  cout << filter << endl;
  istringstream iss(filter);
  do {
    string substr;
    iss >> substr;
    if(!substr.empty()){
      bool flag = !(substr[0]=='-');
      if(substr[0]=='+' || substr[0]=='-')
        substr = substr.substr(1);
      if(!substr.empty()){
        if(flag){
          accept += substr;
          //          cout << "accept: " << substr << endl;
        }else{
          ignore += substr;
          //          cout << "ignore: " << substr << endl;
        }
      }
    }
  } while (iss);
}

bool PlotOption::useChannel(const string& name){
  bool rv= accept.size()==0 ? true : false;
  for (auto& r:accept){
    if (name.find(r)==0){
      rv=true;
      break;
    }
  }
  for (auto& r:ignore){
    if (name.find(r)==0){
      rv=false;
    }
  }
  return rv;
}

int PlotOption::printInspectableNames(const list<const Inspectable*>& inspectables, int cnt) {
  if (!pipe)
    return cnt;
  // here we also set the mask array
  FOREACHC(list<const Inspectable*>, inspectables, insp){
    if(*insp){
      // then the internal parameters
      list<Inspectable::iparamkey> l = (*insp)->getInternalParamNames();
      for(list<Inspectable::iparamkey>::iterator i = l.begin(); i != l.end(); i++){
        const string& str = (*i);
        if((int)mask.size()<=cnt) {
          mask.resize(cnt*2);
        }
        if(useChannel(str)){
          fprintf(pipe, " %s", str.c_str());
          mask[cnt]=true;
        }else
          mask[cnt]=false;
        cnt++;
      }
      cnt += printInspectableNames((*insp)->getInspectables(),cnt);
    }
  }
  return cnt;
}

int PlotOption::printInspectables(const std::list<const Inspectable*>& inspectables, int cnt)
{
  if (!pipe)
    return cnt;

  // internal parameters ( we allocate one place more to be able to realise when the number raises)
  Inspectable::iparamvallist l;
  FOREACHC(list<const Inspectable*>, inspectables, insp)
  {
    if(*insp)
    {
      l = (*insp)->getInternalParams();
      FOREACHC(Inspectable::iparamvallist, l, i )
      {
        if(cnt >= (int)mask.size() || cnt<0) {
          fprintf(stderr, "PlotOption: mask to short: %lu <= %i", mask.size(),cnt); // should not happen
        }else{
          if(mask[cnt])
            fprintf(pipe, " %f", (*i));
        }
        cnt++;
      }
      cnt += printInspectables((*insp)->getInspectables(), cnt);
    }
  }
  return cnt;
}

void PlotOption::printInspectableInfoLines(const list<const Inspectable*>& inspectables) {
  if (!pipe)
    return;
  FOREACHC(list<const Inspectable*>, inspectables, insp) {
    const list<string>& infoLines = (*insp)->getInfoLines();
    FOREACHC(list<string>, infoLines, infoLine) {
      fprintf(pipe,"#I [%s] %s\n", (*insp)->getNameOfInspectable().c_str(), (*infoLine).c_str());
    }
    printInspectableInfoLines((*insp)->getInspectables());
  }
}



void PlotOption::printNetworkDescription(const string& name, const Inspectable* inspectable){
  assert(inspectable);
  if (!pipe)
    return;
  fprintf(pipe,"#N neural_net %s\n", name.c_str());
  list< Inspectable::ILayer> layers      = inspectable->getStructuralLayers();
  list< Inspectable::IConnection> conns  = inspectable->getStructuralConnections();
  // print layers with neurons
  for(list<Inspectable::ILayer>::iterator i = layers.begin(); i != layers.end(); i++){
    Inspectable::ILayer& l = (*i);
    fprintf(pipe, "#N layer %s %i\n", l.layername.c_str(), l.rank);
    for(int n = 0; n < l.dimension; n++){
      if(l.biasname.empty()){
        fprintf(pipe, "#N neuron %s[%i]\n", l.vectorname.c_str(), n);
      }else {
        fprintf(pipe, "#N neuron %s[%i] %s[%i]\n", l.vectorname.c_str(), n, l.biasname.c_str(), n);
      }
    }
  }

  // print connections
  for(list<Inspectable::IConnection>::iterator i = conns.begin(); i != conns.end(); i++){
    Inspectable::IConnection& c = (*i);
    // find the layers refered in the connection description
    list<Inspectable::ILayer>::iterator l1it
      = find_if(layers.begin(), layers.end(), Inspectable::matchName(c.vector1) );
    list<Inspectable::ILayer>::iterator l2it
      = find_if(layers.begin(), layers.end(), Inspectable::matchName(c.vector2) );
    assert(l1it != layers.end()); // we need to find them otherwise
    assert(l2it != layers.end());

    Inspectable::ILayer& l1 = (*l1it);
    Inspectable::ILayer& l2 = (*l2it);
    for(int j=0; j < l1.dimension; j++){
      for(int k=0; k < l2.dimension; k++){
        fprintf(pipe, "#N connection %s[%i,%i] %s[%i] %s[%i]\n",
                c.matrixname.c_str(), k, j, l1.vectorname.c_str(), j, l2.vectorname.c_str(), k);
      }
    }
  }
  fprintf(pipe,"#N nn_end\n");

}
