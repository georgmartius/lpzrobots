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

#include "Gen.h"
#include "restore.h"

Gen::Gen(void) {
  // nothing
}

Gen::Gen(GenPrototype* prototype, int id) {
  m_prototype = prototype;
  m_value = NULL;
  m_ID = id;
}

Gen::~Gen(void) {
  delete m_value;
  m_value = NULL;
}

std::string Gen::getName(void)const {
  return m_prototype->getName();
}

GenPrototype* Gen::getPrototype(void)const {
  return m_prototype;
}

std::string Gen::toString(bool onlyValue)const {
  std::string result = "";

  if(!onlyValue) {
    char buffer[128];

    sprintf(buffer, "%i", m_ID);

    result += "\"" + getName() + "\",\t" + buffer + ",\t";
  }

  result += (std::string)(*m_value);

  return result;
}

bool Gen::store(FILE* f)const {
  RESTORE_GA_GENE head;
  RESTORE_GA_TEMPLATE<int> integer;

  //test
  if(f==NULL) {
    printf("\n\n\t>>> [ERROR] <<<\nNo File to store GA [gene].\n\t>>> [END] <<<\n\n\n");
    return false;
  }

  head.ID = m_ID;

  integer.value=(int)m_prototype->getName().length();
  for(unsigned int d=0;d<sizeof(RESTORE_GA_TEMPLATE<int>);d++) {
    fprintf(f,"%c",integer.buffer[d]);
  }
  fprintf(f,"%s",m_prototype->getName().c_str());

  for(unsigned int x=0;x<sizeof(RESTORE_GA_GENE);x++) {
    fprintf(f,"%c",head.buffer[x]);
  }

  return m_value->store(f);
}
