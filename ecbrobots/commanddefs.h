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
 *   Revision 1.1  2008-04-08 08:14:30  guettler
 *   Initial revision
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __COMMANDDEFS_H
#define __COMMANDDEFS_H

/// type of different commands
/// forbidden: 0xFF (this is the start byte)
#define CDUM   0x0  /* 00000000 Dummy command, do not use this!          */
#define CRES   0x1  /* 00000001 Reset command                            */
#define CDIM   0x2  /* 00000010 Dimension data: number of sensors/motors */
#define CSEN   0x3  /* 00000011 Sensor data values                       */
#define CMOT   0x4  /* 00000100 Motor data values                        */
#define CBEEP  0x8  /* 00001000 Make a beep                              */
#define CMSG   0x9  /* 00001001 Message data.                            */
#define CTEST 0x10  /* 00001010 Test i2c communication.                  */


#endif
