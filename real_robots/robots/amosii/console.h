/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   $Log: console.h,v $
 *   Revision 1.1  2007/10/12 15:27:42  martius
 *   simple test for controllers
 *
 *   Revision 1.1  2007/07/10 13:42:00  robot3
 *   added readline console
 *
 *   Revision 1.1  2007/03/26 13:06:02  martius
 *   new commandline interface
 *
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __CONSOLE_H
#define __CONSOLE_H

#include "globaldata.h"

  /// should be called at the start
  void initializeConsole();
  /// should be called at the end (to store history)
  void closeConsole();

  /** offers a console interface
      the possibility to change parameter of all configurable objects in globalData
      storeing and restoreing of controllers ...
      also informs agents about changes 
      @return false if program should exit
  */
  bool handleConsole(GlobalData& globalData);


#endif
