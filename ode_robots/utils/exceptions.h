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
 *   Revision 1.2  2005-11-09 13:31:51  martius
 *   GPL'ised
 *
 ***************************************************************************/
#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

#define EXCEPTION_TEMPLATE(class_name)                                        \
  class class_name : public IException {                                      \
   public:                                                                    \
    inline virtual void raise() const { throw *this; }                        \
  };


namespace university_of_leipzig {
namespace robots {


class IException {
 public:
  virtual void raise() const = 0;
};


EXCEPTION_TEMPLATE(IndexOutOfBoundsException);
EXCEPTION_TEMPLATE(InvalidArgumentException);
EXCEPTION_TEMPLATE(DimensionMismatchException);



}
}

#endif
