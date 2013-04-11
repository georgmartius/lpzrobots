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

#include "inspectableproxy.h"
#include <string>

InspectableProxy::InspectableProxy(const iparamkey& name) : Inspectable(name) {
        // nothing
}

InspectableProxy::InspectableProxy(const std::list<Inspectable*>& list, const iparamkey& name) : Inspectable(name) {
        //add all parameters of the inspectable in the own list.
        for(std::list<Inspectable*>::const_iterator iter = list.begin(); iter!=list.end(); iter++) {
                //m_list.push_back(*iter);
                std::list<std::string> names = (*iter)->getInternalParamNames();
                std::list<double const*> values = (*iter)->getInternalParamsPtr();
                std::list<std::string>::iterator namesIter = names.begin();
                std::list<double const*>::iterator valuesIter = values.begin();
                unsigned int num = names.size();

                for(unsigned int i = 0; i < num; i++) {
                        addInspectableValue(*namesIter,*valuesIter);
                        namesIter++;
                        valuesIter++;
                }
        }
}

InspectableProxy::~InspectableProxy() {
        // nothing
        //clear all
        /*while(mapOfValues.size()>0){
                std::list<std::pair<std::string,double*> >::iterator i = mapOfValues.begin();
                free(const_cast<std::string>(i->first));
                i->second = 0;
                mapOfValues.erase(i);
        }*/
        /*m_list.clear();
        mapOfValues.clear();
        mapOfMatrices.clear();*/
}

bool InspectableProxy::replaceList(const std::list<Inspectable*>& list) {
        //needs the same parameter count
        //if(m_list.size() != list.size())
                //return false;

        //clear all
        /*while(mapOfValues.size()>0){
                std::list<std::pair<std::string,double*> >::iterator i = mapOfValues.begin();
                free(const_cast<std::string>(i->first));
                i->second = 0;
                mapOfValues.erase(i);
        }*/
        /*m_list.clear();
        mapOfValues.clear();
        mapOfMatrices.clear();

        //add the new parameters
        for(std::list<Inspectable*>::const_iterator iter = list.begin(); iter!=list.end(); iter++) {
                m_list.push_back(*iter);
                std::list<std::string> names = (*iter)->getInternalParamNames();
                std::list<double*> values = (*iter)->getInternalParamsPtr();
                std::list<std::string>::iterator namesIter = names.begin();
                std::list<double*>::iterator valuesIter = values.begin();
                unsigned int num = names.size();

                for(unsigned int i = 0; i < num; i++) {
                        addInspectableValue(*namesIter,*valuesIter);
                        namesIter++;
                        valuesIter++;
                }
        }*/

        FOREACHC(std::list<Inspectable*>,list,i) {
                std::list<std::string> names = (*i)->getInternalParamNames();
                std::list<double const *> values = (*i)->getInternalParamsPtr();
                std::list<double const*>::iterator l = values.begin();

                FOREACH(std::list<std::string>,names,j) {
                        FOREACH(Inspectable::iparampairlist,mapOfValues,k) {
                                if(!k->first.compare(*j)){
                                        k->second = *l;
                                        break;
                                }
                        }
                        l++;
                }
        }

        return true;
}
