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
 ** Started on  Mon Oct 15 18:01:12 2007 Georg Martius
 ** Last update Mon Oct 15 18:01:12 2007 Georg Martius
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2007-11-07 13:17:40  martius
 *   motors in a general sense (like sound, light,...)
 *
 *
 *                                                                 *
 ***************************************************************************/
#ifndef   	DUMMYMOTOR_H_
# define   	DUMMYMOTOR_H_

#include "motor.h"

namespace lpzrobots {
  
  class DummyMotor: public Motor {
  public: 
    DummyMotor(int number=1)
      : number(number) {
    }
    virtual ~DummyMotor() {};
    
    virtual void init(Primitive* own){
    }

    virtual int getMotorNumber() const{
      return number;
    };

    virtual bool act(GlobalData& globaldata){
      return true;
    }

    virtual int set(const motor* values, int length){
      return number;
    };    
    
  private:
    int number;
  };

}

#endif 	    /* !DUMMYMOTOR_H_ */
