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

#include "configurable.h"
#include "inspectable.h"
#include <cstring>
#include <assert.h>
#include <stdio.h>
#include <cmath>
#include <locale.h> // need to set LC_NUMERIC to have a '.' in the numbers written

using namespace std;

#ifndef AVR

bool Configurable::storeCfg(const char* filenamestem,
                const std::list< std::string>& comments){
  char name[256];
  FILE* f;
  setlocale(LC_NUMERIC,"en_US"); // set us type output

  sprintf(name, "%s",filenamestem);
  if(!(f = fopen(name, "w"))) return false;
  FOREACHC(std::list< std::string>,comments,c){
    fprintf(f, "# %s\n", c->c_str());
  }
  print(f,0); // save config values to file
  fclose(f);
  return true;
}

bool Configurable::restoreCfg(const char* filenamestem){
  char name[256];
  FILE* f;
  setlocale(LC_NUMERIC,"en_US"); // set us type output

  sprintf(name, "%s",filenamestem);
  if(!(f=fopen(name, "r")))
    return false;
  bool rv = parse(f);
  fclose(f);
  return rv;
}


Configurable::paramval Configurable::getParam(const paramkey& key, bool traverseChildren) const {
  parammap::const_iterator it = mapOfValues.find(key);
  if (it != mapOfValues.end()) {
    return *((*it).second);
  } else {
    // now try to find in map for int values
    paramintmap::const_iterator intit = mapOfInteger.find(key);
    if (intit != mapOfInteger.end()) {
      return (paramval) *((*intit).second);
    } else {
      // now try to find in map for boolean values
      paramboolmap::const_iterator boolit = mapOfBoolean.find(key);
      if (boolit != mapOfBoolean.end()) {
        return (paramval) *((*boolit).second);
      } else {
        if (traverseChildren) {
          // search in all configurable children
          FOREACHC(configurableList, ListOfConfigurableChildren, conf) {
            if ((*conf)->hasParam(key))
              return ((*conf)->getParam(key));
          }
        }
        std::cerr << name << ": " << __FUNCTION__ << ": parameter " << key << " unknown\n";
        return 0;
      }
    }
  }
}

bool Configurable::hasParam(const paramkey& key, bool traverseChildren) const {
  if(mapOfValues.find(key) != mapOfValues.end()
             || mapOfInteger.find(key) != mapOfInteger.end()
             || mapOfBoolean.find(key) != mapOfBoolean.end())
    return true;
  if (traverseChildren) {
    // search in all configurable children
    FOREACHC(configurableList, ListOfConfigurableChildren, conf) {
      if ((*conf)->hasParam(key))
        return true;
    }
  }
  return false;
}

bool Configurable::setParam(const paramkey& key, paramval val, bool traverseChildren) {
  parammap::const_iterator it = mapOfValues.find(key);
  bool valueSet = false;
  if (it != mapOfValues.end()) {
    *(mapOfValues[key]) = val;
    valueSet = true;
  } else {
    // now try to find in map for boolean values
    paramintmap::const_iterator intit = mapOfInteger.find(key);
    if (intit != mapOfInteger.end()) {
      *(mapOfInteger[key]) = (int) val;
      valueSet = true;
    } else {
      // now try to find in map for boolean values
      paramboolmap::const_iterator boolit = mapOfBoolean.find(key);
      if (boolit != mapOfBoolean.end()) {
        *(mapOfBoolean[key]) = val != 0 ? true : false;
        valueSet = true;
      }
    }
  }
  if(valueSet) notifyOnChange(key);
  // search in all configurable children
  if (traverseChildren) {
    FOREACHC(configurableList, ListOfConfigurableChildren, conf) {
      //      if ((*conf)->hasParam(key)) // not needed
      bool vs = ((*conf)->setParam(key, val));
      valueSet |= vs;
    }
  }
  return valueSet;
}


std::list<Configurable::paramkey> Configurable::getAllParamNames(bool traverseChildren){
  std::list<paramkey> l;
  FOREACHC(parammap, mapOfValues, i) {
     l += (*i).first;
  }
  FOREACHC(paramintmap, mapOfInteger, i) {
    l += (*i).first;
  }
  FOREACHC(paramboolmap, mapOfBoolean, i) {
    l += (*i).first;
  }
  // add custom parameters stuff (which is marked by a * at the end of the line)
  paramlist list = getParamList();
  FOREACHC(paramlist, list, i) {
    l += (*i).first;
  }
  if (traverseChildren) {
    // add all parameters from the configurable children
    FOREACHC(configurableList, ListOfConfigurableChildren, conf) {
      l+= (*conf)->getAllParamNames();
    }
  }
  return l;
}

Configurable::paramdescr Configurable::getParamDescr(const paramkey& key, bool traverseChildren) const {
  paramdescrmap::const_iterator it = mapOfDescr.find(key);
  if (it != mapOfDescr.end()) {
    return it->second;
  }else if (traverseChildren) {
    FOREACHC(configurableList, ListOfConfigurableChildren, conf) {
      if ((*conf)->hasParamDescr(key))
        return ((*conf)->getParamDescr(key));
    }
  }
  return paramdescr();
}

bool Configurable::hasParamDescr(const paramkey& key, bool traverseChildren) const {
  if (mapOfDescr.find(key)!=mapOfDescr.end())
      return true;
  if (traverseChildren) {
    FOREACHC(configurableList, ListOfConfigurableChildren, conf) {
      if ((*conf)->hasParamDescr(key))
        return true;
    }
  }
  return false;
}

// copies the internal params of the given configurable
void Configurable::copyParameters(const Configurable& c, bool traverseChildren){
  mapOfValues  = c.mapOfValues;
  mapOfBoolean = c.mapOfBoolean;
  mapOfInteger = c.mapOfInteger;
  mapOfDescr   = c.mapOfDescr;

  mapOfValBounds = c.mapOfValBounds;
  mapOfIntBounds = c.mapOfIntBounds;
  if (traverseChildren) {
    ListOfConfigurableChildren = c.ListOfConfigurableChildren;
    parent = c.parent;
  }
}

Configurable::paramvalBounds Configurable::getParamvalBounds(const paramkey& key, bool traverseChildren) const {
  paramvalBoundsMap::const_iterator it = mapOfValBounds.find(key);
  if (it != mapOfValBounds.end()) {
    return it->second;
  }else if (traverseChildren) {
    FOREACHC(configurableList, ListOfConfigurableChildren, conf) {
      if ((*conf)->hasParamvalBounds(key))
        return (*conf)->getParamvalBounds(key);
    }
  }
  return paramvalBounds(valDefMinBound,valDefMaxBound);
}

bool Configurable::hasParamvalBounds(const paramkey& key, bool traverseChildren) const {
  if (mapOfValBounds.find(key)!=mapOfValBounds.end())
    return true;
  if (traverseChildren) {
    FOREACHC(configurableList, ListOfConfigurableChildren, conf) {
      if ((*conf)->hasParamvalBounds(key))
        return true;
    }
  }
  return false;
}

Configurable::paramintBounds Configurable::getParamintBounds(const paramkey& key, bool traverseChildren) const {
  paramintBoundsMap::const_iterator it = mapOfIntBounds.find(key);
  if (it != mapOfIntBounds.end()) {
    return it->second;
  }else if (traverseChildren) {
    FOREACHC(configurableList, ListOfConfigurableChildren, conf) {
      if ((*conf)->hasParamintBounds(key))
        return (*conf)->getParamintBounds(key);
    }
  }
  return paramintBounds(intDefMinBound,intDefMaxBound);
}


bool Configurable::hasParamintBounds(const paramkey& key, bool traverseChildren) const {
  if (mapOfIntBounds.find(key)!=mapOfIntBounds.end())
    return true;
  if (traverseChildren) {
    FOREACHC(configurableList, ListOfConfigurableChildren, conf) {
      if ((*conf)->hasParamintBounds(key))
        return true;
    }
  }
  return false;
}


void Configurable::setParamBounds(const paramkey& key, paramval minBound, paramval maxBound, bool traverseChildren) {
  if (mapOfValBounds.find(key)!=mapOfValBounds.end())
    mapOfValBounds[key]=paramvalBounds(minBound,maxBound);
  if (traverseChildren) {
    FOREACH(configurableList, ListOfConfigurableChildren, conf) {
      if ((*conf)->hasParamvalBounds(key))
        (*conf)->setParamBounds(key, minBound, maxBound);
    }
  }
}

void Configurable::setParamBounds(const paramkey& key, paramint minBound, paramint maxBound, bool traverseChildren) {
  if (mapOfIntBounds.find(key)!=mapOfIntBounds.end())
    mapOfIntBounds[key]=paramintBounds(minBound,maxBound);
  if (traverseChildren) {
    FOREACH(configurableList, ListOfConfigurableChildren, conf) {
      if ((*conf)->hasParamintBounds(key))
        (*conf)->setParamBounds(key, minBound, maxBound);
    }
  }
}

void Configurable::setParamBounds(const paramkey& key, paramvalBounds bounds, bool traverseChildren) {
  setParamBounds(key, bounds.first, bounds.second, traverseChildren);
}

void Configurable::setParamBounds(const paramkey& key, paramintBounds bounds, bool traverseChildren) {
  setParamBounds(key, bounds.first, bounds.second, traverseChildren);
}

void Configurable::setParamDescr(const paramkey& key, const paramdescr& descr, bool traverseChildren) {
  if (hasParam(key, false)) {
    if(!descr.empty())
      mapOfDescr[key] = descr;
    else // delete entry if exist
      mapOfDescr.erase(key);
  } if (traverseChildren) {
    FOREACH(configurableList, ListOfConfigurableChildren, conf) {
      if ((*conf)->hasParam(key))
        (*conf)->setParamDescr(key, descr);
    }
  }
}



void Configurable::print(FILE* f, const char* prefix, int columns, bool traverseChildren /* = true */) const {
  const char* pre = prefix==0 ? "": prefix;
  columns-= strlen(pre);
  const unsigned short spacelength=20;
  char spacer[spacelength+1];
  memset(spacer, ' ', spacelength);  spacer[spacelength]=0;
  // print header
  fprintf(f, "%s[%s, %s][%i]\n", pre, getName().c_str(), getRevision().c_str(), getId());
  // use map of values
  FOREACHC(parammap, mapOfValues, i) {
    const string& k = (*i).first;
    double val = * (*i).second;
    if(val>1000 && floor(val) == val){ // without point and digits afterwards
      fprintf(f, "%s %s=%s%11.0f ", pre, k.c_str(),
              spacer+(k.length() > spacelength  ? spacelength : k.length()), * (*i).second);
    }else{ // normal
      fprintf(f, "%s %s=%s%11.6f ", pre, k.c_str(),
              spacer+(k.length() > spacelength  ? spacelength : k.length()), * (*i).second);
    }
    printdescr(f, pre, k, columns,spacelength+13);
  }
  // use map of int
  FOREACHC(paramintmap, mapOfInteger, i) {
    const string& k = (*i).first;
    fprintf(f, "%s %s=%s%4i        ", pre, k.c_str(),
            spacer+(k.length() > spacelength  ? spacelength : k.length()), * (*i).second);
    printdescr(f, pre, k, columns,spacelength+13);

  }
  // use map of boolean
  FOREACHC(paramboolmap, mapOfBoolean, i) {
    const string& k = (*i).first;
    fprintf(f, "%s %s=%s%4i        ", pre, k.c_str(),
      spacer+(k.length() > spacelength  ? spacelength : k.length()), * (*i).second);
    printdescr(f, pre, k, columns,spacelength+13);
  }
  // add custom parameters stuff (which is marked by a * at the end of the line)
  paramlist list = getParamList();
  FOREACHC(paramlist, list, i) {
    const string& k = (*i).first;
    fprintf(f, "%s %s=%s%11.6f*", pre, k.c_str(),
            spacer+(k.length() > spacelength  ? spacelength : k.length()), (*i).second);
    printdescr(f, pre, k, columns,spacelength+13);
  }
  // write termination line
  fprintf(f, "%s######\n",pre);
  // print also all registered configurable children
  if (traverseChildren) {
    char newPrefix[strlen(pre)+2];
    sprintf(newPrefix,"%s-",pre);
    FOREACHC(configurableList, ListOfConfigurableChildren, conf) {
      (*conf)->print(f, newPrefix, columns, true);
    }
  }
}

void Configurable::printdescr(FILE* f, const char* prefix,
                              const paramkey& key, int columns, int indent) const {
  paramdescr descr = getParamDescr(key);
  int len = descr.length();
  int space = max(columns - indent,5);
  // maybe one can also split at the word boundaries
  while(len > space){
    const paramdescr& d = descr.substr(0,space);
    char* spacer = new char[indent+1];
    memset(spacer, ' ', indent);  spacer[indent]=0;

    fprintf(f, " %s\n%s %s",d.c_str(),prefix, spacer);
    delete[] spacer;
    descr = descr.substr(space,len);
    len-=space;
  };
  fprintf(f, " %s\n",descr.c_str());
}


bool Configurable::parse(FILE* f, const char* prefix, bool traverseChildren) {
  char* buffer = (char*)malloc(sizeof(char)*512);
  int preLen= prefix==0 ? 0 : strlen(prefix);
  char pre[preLen+2];
  bool rv=true;
  if(prefix!=0)
    strcpy(pre,prefix);

  assert(buffer);
  while (fgets(buffer, 512, f) != 0) {
    char* bufNoPrefix = buffer;
    if(preLen>0){
      if(strncmp(buffer,pre,preLen)==0)
        bufNoPrefix = buffer+preLen;
      else {
        fprintf(stderr,"could not detect prefix: %s in line: %s\n", pre,buffer);
        rv=false;
        break;
      }
    }
    if(strcmp(bufNoPrefix,"######\n")==0) break;
    char* p = strchr(bufNoPrefix,'=');
    if(p!=0){
      *p=0; // terminate string (key) at = sign
      char* s = bufNoPrefix;
      while (*s==' ') s++; // skip leading spaces
      // cout << "Set: " << s << " Val:" << atof(p+1) << endl;
      setParam(s, atof(p+1),false);
    }
  }
  free(buffer);
  if (traverseChildren) {
    pre[preLen]='-';
    pre[preLen+1]=0;
    FOREACH(configurableList, ListOfConfigurableChildren, conf) {
      (*conf)->parse(f, pre, true);
    }
  }else{ // we would need to read all prefixed lines of possible children, if there are any
    // but we do not know when to stop and would read too much
  }
  return rv;
}

void Configurable::addConfigurable(Configurable* conf) {
  ListOfConfigurableChildren.push_back(conf);
  conf->parent=this;
}

void Configurable::removeConfigurable(Configurable* conf) {
  removeElement(ListOfConfigurableChildren, conf);
  conf->parent=0;
}

const Configurable::configurableList& Configurable::getConfigurables() const {
  return ListOfConfigurableChildren;
}

void Configurable::configurableChanged() {
  callBack(CALLBACK_CONFIGURABLE_CHANGED);
}

void Configurable::setName(const paramkey& name, bool callSetNameOfInspectable) {
  this->name = name;
  Inspectable* insp = dynamic_cast<Inspectable*>(this);
  if (insp)
    insp->setNameOfInspectable(name);
}



#endif
