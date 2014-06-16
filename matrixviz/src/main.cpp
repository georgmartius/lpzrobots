/***************************************************************************
 *   Copyright (C) 2008-2011 LpzRobots development team                    *
 *    Antonia Siegert (original author)                                  *
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

#include <signal.h>
#include <QApplication>
#include "MatrixVisualizer.h"

void signal_handler_exit(void){
  signal(SIGINT,SIG_DFL);
}

void control_c(int ){ }

/**
 * We need to catch Ctrl-C (SIGINT) because if we are called from another program (like ode simulation) that reacts on Ctrl-C we are killed.
 * SIGPIPE is emitted if the stdin or stdout breaks.
 * We need to terminate if the stdin breaks to get closed with the calling application (in  pipe mode).
*/
void signal_handler_init(){
  signal(SIGINT,control_c);
  atexit(signal_handler_exit);
  signal(SIGPIPE, SIG_DFL);
}

int contains(char **list, int len,  const char *str) {
  for(int i=0; i<len; i++) {
    if(strcmp(list[i],str) == 0)
      return i+1;
  }
  return 0;
}

int main(int argc, char *argv[])
{


  if(contains(argv,argc,"--help")!=0 || contains(argv,argc,"-h")!=0){
    printf("Usage: yourprog | %s [-noCtrlC] [-novideo]\n",argv[0]);
    printf("\t-noCtrlC will catch ctrlC and the program only terminates via the pipe\n");
    printf("\t-novideo will not record frames on #V lines (because it slows down significantly)\n");
    return 0;
  }

  if(contains(argv,argc,"-noCtrlC"))
    signal_handler_init();
  bool novideo = contains(argv,argc,"-novideo")!=0;

  QApplication app(argc, argv);
  MatrixVisualizer gui(0, novideo);
  int rv = app.exec();

  signal_handler_exit();
  return rv;
}
