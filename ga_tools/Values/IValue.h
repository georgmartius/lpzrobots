/***************************************************************************
 *   Copyright (C) 2008-2011 LpzRobots development team                    *
 *    Joerg Weider   <joergweide84 at aol dot com> (robot12)               *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *    Joern Hoffmann <jhoffmann at informatik dot uni-leipzig dot de       *
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
#ifndef IVALUE_H_
#define IVALUE_H_

//includes
#include <string>
#include <selforg/inspectable.h>
#include <selforg/storeable.h>

/**
 * This class is a interface for a value which is part of a gen. Over this concept is it paissible
 * to make the Gen and the GenFactory independent from his saved type.
 */
class IValue : public Inspectable
{
public:

  /**
   * constructor
   * Needs a string for the name of the value. In the normal way it is the Type of the Value. For example "templateValue".
   * @param name (string) the name
   */
  IValue(std::string name);


  /**
   * default destructor
   */
  virtual ~IValue();

  /**
   * the mul. operator. Dosn't change this class!!!
   * @param (const IValue&) the other part of the operation
   * @return (IValue*) the result
   */
  virtual IValue* operator*(const IValue&)const = 0;

   /**
	* the add operator. Dosn't change this class!!!
	* @param (const IValue&) the other part of the operation
	* @return (IValue*) the result
	*/
  virtual IValue* operator+(const IValue&)const = 0;

  /**
   * the cast operator for a cast in type string
   * @return (string) the value as string
   */
  virtual operator std::string(void)const;

  /**
   * store the value in a file
   * @param f (FILE*) the file to store
   * @return (bool) true if all ok.
   */
  virtual bool store(FILE* f) const;

  /**
   * restore the value from a file
   * @param f (FILE*) the file where the value inside
   * @return (bool) true if all ok.
   */
  virtual bool restore(FILE* f);

protected:
	/**
	 * the name of this class.
	 */
	std::string m_name;
};

#endif /* IVALUE_H_ */
