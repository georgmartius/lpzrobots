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

#ifndef __BACKCALLERVECTOR_H_
#define __BACKCALLERVECTOR_H_

#include <vector>

/**
 * Establishes for some methods the notifications for registered Callbackable instances
 * (use addCallbackable(...)).
 * @warning Only the following methods are currently supported: 
    push_back(...), pop_back(), erase() and clear()!
 * You can use iterators with the limitation to not delete or insert.
 */
template<typename _Tp, typename _Alloc = std::allocator<_Tp> >
class BackCallerVector : public std::vector<_Tp,_Alloc>, public BackCaller {
public:

  typedef typename std::vector<_Tp,_Alloc>::iterator iterator;

  BackCallerVector() {}
  virtual ~BackCallerVector(){
    callBack(BACKCALLER_VECTOR_BEING_DELETED);
  }
  
  /**
   * Indicates that the list/vector has been modified, 
   a new instance was either added or removed.
  */
  static const CallbackableType BACKCALLER_VECTOR_MODIFIED = 10219;
  
  /**
   * Indicates that the list is being deleted.
   */
  static const CallbackableType BACKCALLER_VECTOR_BEING_DELETED = 10220;


  void push_back(const _Tp & i){
    std::vector<_Tp,_Alloc>::push_back(i);
    callBack(BACKCALLER_VECTOR_MODIFIED);
  }
  
  iterator erase(iterator pos){
    iterator i  = std::vector<_Tp,_Alloc>::erase(pos);
    callBack(BACKCALLER_VECTOR_MODIFIED);
    return i;  
  }
  
  void pop_back(){
    std::vector<_Tp,_Alloc>::pop_back();
    callBack(BACKCALLER_VECTOR_MODIFIED);
  }
  
  void clear(){
    std::vector<_Tp,_Alloc>::clear();
    callBack(BACKCALLER_VECTOR_MODIFIED);
  }
};

#endif /* __BACKCALLERVECTOR_H_ */

