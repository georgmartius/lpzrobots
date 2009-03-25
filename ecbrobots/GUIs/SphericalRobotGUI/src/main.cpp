/***************************************************************************
 *   Copyright (C) 2008 by mc   *
 *   mc@linux-6hav   *
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
 ***************************************************************************/

#include <signal.h>
#include <QApplication>
#include "SphericalRobotGUI.h"

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

int main(int argc, char *argv[])
{
  
//   signal_handler_init();
  
  QApplication app(argc, argv);
  
    
  
  SphericalRobotGUI gui;
  
  
  gui.show();
  return app.exec();
}

