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
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2008-04-29 07:39:54  guettler
 *   interfaces moved to selforg/utils directory
 *
 *   Revision 1.1  2006/07/19 09:27:00  martius
 *   interface for storeable objects
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __STOREABLE_H
#define __STOREABLE_H

#include<stdio.h>

/**
 * Interface for objects, that can be stored and restored to/from a file stream (binary).
*/

class Storeable {
public:
  virtual ~Storeable(){}
  /** stores the object to the given file stream (binary). 
  */
  virtual bool store(FILE* f) const = 0;
  
  /** loads the object from the given file stream (binary). 
  */
  virtual bool restore(FILE* f) = 0;  
};

#endif
