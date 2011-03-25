/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *
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
 *   $Log$
 *   Revision 1.15  2011-03-25 20:38:46  guettler
 *   - setName() also calls setNameOfInspectable if the instance is inspectable, this
 *     can be avoided manually with an additional param (the autoset should normally
 *     do what the end-user intends because he cannot know that the derived classes
 *     are not overloading this function.)
 *
 *   Revision 1.14  2011/03/22 16:48:57  guettler
 *   - Configurable derives from BackCaller to support method configurableChanged()
 *     for future work with the Configurator GUI (works already in ecb_robots)
 *   - print() now considers the configurable childs
 *
 *   Revision 1.13  2011/03/21 23:08:58  guettler
 *   - setParam etc. now sets all params in child configurables even if set already
 *
 *   Revision 1.12  2011/03/21 17:45:36  guettler
 *   enhanced configurable interface:
 *   - support for configurable childs of a configurable
 *   - some new helper functions
 *
 *   Revision 1.11  2010/12/16 15:26:09  martius
 *   added copyParameter
 *
 *   Revision 1.10  2010/12/06 14:09:53  guettler
 *   - use of valDefMinBounds, valDefMaxBounds, intDefMinBounds and intDefMaxBound instead of inf values
 *
 *   Revision 1.9  2010/11/26 12:15:05  guettler
 *   - Configurable interface now allows to set bounds of paramval and paramint
 *     * setting bounds for paramval and paramint is highly recommended (for QConfigurable (Qt GUI).
 *
 *   Revision 1.8  2010/03/03 14:56:30  martius
 *   improved printing of parameterdescription
 *
 *   Revision 1.7  2010/02/01 15:22:02  guettler
 *   added #include <stdio.h> for compatibility with gcc 4.4
 *
 *   Revision 1.6  2009/10/29 13:14:05  guettler
 *   compile FIX: added #include <assert.h>
 *
 *   Revision 1.5  2009/10/28 17:53:11  fhesse
 *   got rid of getline and replaced it by fgets to be compatible with MAC
 *
 *   Revision 1.4  2009/08/05 22:47:33  martius
 *   added support for integer variables and
 *    proper display for bool and int
 *
 *   Revision 1.3  2009/08/05 08:36:22  guettler
 *   added support for boolean variables
 *
 *   Revision 1.2  2009/03/27 06:16:57  guettler
 *   support for gcc 4.3 compatibility (has to be checked), StatisticTools moves from utils to statistictools
 *
 *   Revision 1.1  2008/04/29 07:39:54  guettler
 *   interfaces moved to selforg/utils directory
 *
 *   Revision 1.7  2008/04/11 14:12:30  martius
 *   comments paramter in storeCfg
 *
 *   Revision 1.6  2007/06/08 15:44:32  martius
 *   termination string -> more robust parsing
 *
 *   Revision 1.5  2007/03/26 13:13:47  martius
 *   store and restore with params
 *
 *   Revision 1.4  2007/01/24 14:38:35  martius
 *   new implementation with a map from key to reference.
 *   addParameter and addParameterDef are used for registration of a parameter
 *   the old style is still supported
 *
 *   Revision 1.3  2006/07/14 12:23:57  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.7  2006/06/25 21:56:07  martius
 *   configureable has name and revision
 *
 *   Revision 1.1.2.6  2006/02/08 16:17:40  martius
 *   no namespace using
 *
 *   Revision 1.1.2.5  2006/01/18 16:48:10  martius
 *   configurables can be stored and reloaded
 *
 *   Revision 1.1.2.4  2006/01/18 10:45:32  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.3  2006/01/18 10:44:49  martius
 *   (re)storeCfg
 *
 *   Revision 1.1.2.2  2006/01/16 17:40:13  martius
 *   parsing works
 *
 *   Revision 1.1.2.1  2006/01/16 17:27:17  martius
 *   *** empty log message ***
 *
 *                                                                 *
 ***************************************************************************/

#include "configurable.h"
#include "inspectable.h"
#include <cstring>
#include <assert.h>
#include <stdio.h>

using namespace std;

void Configurable::insertCVSInfo(paramkey& str, const char* file, const char* revision){
  string f(file);
  string rev(revision);
  int colon = f.find(':');
  int end   = f.rfind('$');
  if(colon >= 0 && end >= 0 && colon+1 < end){
    str += f.substr(colon+1, end-colon-1);
  }
  str += "-";
  colon = rev.find(':');
  end   = rev.rfind('$');
  if(colon >= 0 && end >= 0 && colon+1 < end){
    str += rev.substr(colon+1, end-colon-1);
  }
}

#ifndef AVR

bool Configurable::storeCfg(const char* filenamestem,
		const std::list< std::string>& comments){
  char name[256];
  FILE* f;
  sprintf(name, "%s.cfg",filenamestem);
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
  sprintf(name, "%s.cfg",filenamestem);
  if(!(f=fopen(name, "r")))
    return false;
  parse(f);
  fclose(f);
  return true;
}


Configurable::paramval Configurable::getParam(const paramkey& key, bool useChilds) const {
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
        if (useChilds) {
          // search in all configurable childs
          FOREACHC(configurableList, ListOfConfigurableChilds, conf) {
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

bool Configurable::hasParam(const paramkey& key, bool useChilds) const {
  if(mapOfValues.find(key) != mapOfValues.end()
             || mapOfInteger.find(key) != mapOfInteger.end()
             || mapOfBoolean.find(key) != mapOfBoolean.end())
    return true;
  if (useChilds) {
    // search in all configurable childs
    FOREACHC(configurableList, ListOfConfigurableChilds, conf) {
      if ((*conf)->hasParam(key))
        return true;
    }
  }
  return false;
}

bool Configurable::setParam(const paramkey& key, paramval val, bool useChilds) {
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
  // search in all configurable childs
  if (useChilds) {
    FOREACHC(configurableList, ListOfConfigurableChilds, conf) {
      if ((*conf)->hasParam(key))
        valueSet |= ((*conf)->setParam(key, val));
    }
  }
  return valueSet;
}


std::list<Configurable::paramkey> Configurable::getAllParamNames(bool useChilds){
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
  if (useChilds) {
    // add all parameters from the configurable childs
    FOREACHC(configurableList, ListOfConfigurableChilds, conf) {
      l+= (*conf)->getAllParamNames();
    }
  }
  return l;
}

Configurable::paramdescr Configurable::getParamDescr(const paramkey& key, bool useChilds) const {
  paramdescrmap::const_iterator it = mapOfDescr.find(key);
  if (it != mapOfDescr.end()) {
    return it->second;
  }else if (useChilds) {
    FOREACHC(configurableList, ListOfConfigurableChilds, conf) {
      if ((*conf)->hasParamDescr(key))
        return ((*conf)->getParamDescr(key));
    }
  }
  return paramdescr();
}

bool Configurable::hasParamDescr(const paramkey& key, bool useChilds) const {
  if (mapOfDescr.find(key)!=mapOfDescr.end())
      return true;
  if (useChilds) {
    FOREACHC(configurableList, ListOfConfigurableChilds, conf) {
      if ((*conf)->hasParamDescr(key))
        return true;
    }
  }
  return false;
}

// copies the internal params of the given configurable
void Configurable::copyParameters(const Configurable& c, bool useChilds){
  mapOfValues  = c.mapOfValues;
  mapOfBoolean = c.mapOfBoolean;
  mapOfInteger = c.mapOfInteger;
  mapOfDescr   = c.mapOfDescr;
  
  mapOfValBounds = c.mapOfValBounds;
  mapOfIntBounds = c.mapOfIntBounds;
  if (useChilds) {
    ListOfConfigurableChilds = c.ListOfConfigurableChilds;
    parent = c.parent;
  }
}

Configurable::paramvalBounds Configurable::getParamvalBounds(const paramkey& key, bool useChilds) const {
  paramvalBoundsMap::const_iterator it = mapOfValBounds.find(key);
  if (it != mapOfValBounds.end()) {
    return it->second;
  }else if (useChilds) {
    FOREACHC(configurableList, ListOfConfigurableChilds, conf) {
      if ((*conf)->hasParamvalBounds(key))
        return (*conf)->getParamvalBounds(key);
    }
  }
  return paramvalBounds(valDefMinBound,valDefMaxBound);
}

bool Configurable::hasParamvalBounds(const paramkey& key, bool useChilds) const {
  if (mapOfValBounds.find(key)!=mapOfValBounds.end())
    return true;
  if (useChilds) {
    FOREACHC(configurableList, ListOfConfigurableChilds, conf) {
      if ((*conf)->hasParamvalBounds(key))
        return true;
    }
  }
  return false;
}

Configurable::paramintBounds Configurable::getParamintBounds(const paramkey& key, bool useChilds) const {
  paramintBoundsMap::const_iterator it = mapOfIntBounds.find(key);
  if (it != mapOfIntBounds.end()) {
    return it->second;
  }else if (useChilds) {
    FOREACHC(configurableList, ListOfConfigurableChilds, conf) {
      if ((*conf)->hasParamintBounds(key))
        return (*conf)->getParamintBounds(key);
    }
  }
  return paramintBounds(intDefMinBound,intDefMaxBound);
}


bool Configurable::hasParamintBounds(const paramkey& key, bool useChilds) const {
  if (mapOfIntBounds.find(key)!=mapOfIntBounds.end())
    return true;
  if (useChilds) {
    FOREACHC(configurableList, ListOfConfigurableChilds, conf) {
      if ((*conf)->hasParamintBounds(key))
        return true;
    }
  }
  return false;
}


void Configurable::setParamBounds(const paramkey& key, paramval minBound, paramval maxBound, bool useChilds) {
  if (mapOfValBounds.find(key)!=mapOfValBounds.end())
    mapOfValBounds[key]=paramvalBounds(minBound,maxBound);
  if (useChilds) {
    FOREACH(configurableList, ListOfConfigurableChilds, conf) {
      if ((*conf)->hasParamvalBounds(key))
        (*conf)->setParamBounds(key, minBound, maxBound);
    }
  }
}

void Configurable::setParamBounds(const paramkey& key, paramint minBound, paramint maxBound, bool useChilds) {
  if (mapOfIntBounds.find(key)!=mapOfIntBounds.end())
    mapOfIntBounds[key]=paramintBounds(minBound,maxBound);
  if (useChilds) {
    FOREACH(configurableList, ListOfConfigurableChilds, conf) {
      if ((*conf)->hasParamintBounds(key))
        (*conf)->setParamBounds(key, minBound, maxBound);
    }
  }
}

void Configurable::setParamBounds(const paramkey& key, paramvalBounds bounds, bool useChilds) {
  setParamBounds(key, bounds.first, bounds.second, useChilds);
}

void Configurable::setParamBounds(const paramkey& key, paramintBounds bounds, bool useChilds) {
  setParamBounds(key, bounds.first, bounds.second, useChilds);
}

void Configurable::setParamDescr(const paramkey& key, const paramdescr& descr, bool useChilds) {
  if (hasParam(key, false)) {
    if(!descr.empty())
      mapOfDescr[key] = descr;
    else // delete entry if exist
      mapOfDescr.erase(key);
  } if (useChilds) {
    FOREACH(configurableList, ListOfConfigurableChilds, conf) {
      if ((*conf)->hasParam(key))
        (*conf)->setParamDescr(key, descr);
    }
  }
}



void Configurable::print(FILE* f, const char* prefix, int columns, bool useChilds /* = true */) const {
  const char* pre = prefix==0 ? "": prefix;    
  const unsigned short spacelength=20;
  char spacer[spacelength+1];
  memset(spacer, ' ', spacelength);  spacer[spacelength]=0;
  // print header
  fprintf(f, "%s[%s, %s][%i]\n", pre, getName().c_str(), getRevision().c_str(), getId());
  // use map of values
  FOREACHC(parammap, mapOfValues, i) {
    const string& k = (*i).first;
    fprintf(f, "%s %s=%s%11.6f  ", pre, k.c_str(), 
	    spacer+(k.length() > spacelength  ? spacelength : k.length()), * (*i).second);
    printdescr(f, pre, k, columns,spacelength+14);
  }
  // use map of int
  FOREACHC(paramintmap, mapOfInteger, i) {
    const string& k = (*i).first;
    fprintf(f, "%s %s=%s%4i         ", pre, k.c_str(),
            spacer+(k.length() > spacelength  ? spacelength : k.length()), * (*i).second);
    printdescr(f, pre, k, columns,spacelength+14);

  }
  // use map of boolean
  FOREACHC(paramboolmap, mapOfBoolean, i) {
    const string& k = (*i).first;
    fprintf(f, "%s %s=%s%4i         ", pre, k.c_str(),
      spacer+(k.length() > spacelength  ? spacelength : k.length()), * (*i).second);
    printdescr(f, pre, k, columns,spacelength+14);
  }
  // add custom parameters stuff (which is marked by a * at the end of the line)
  paramlist list = getParamList(); 
  FOREACHC(paramlist, list, i) {
    const string& k = (*i).first;
    fprintf(f, "%s %s=%s%11.6f *", pre, k.c_str(), 
	    spacer+(k.length() > spacelength  ? spacelength : k.length()), (*i).second);
    printdescr(f, pre, k, columns,spacelength+14);
  }
  // write termination line
  fprintf(f, "######\n");
  // print also all registered configurable childs
  if (useChilds) {
    FOREACHC(configurableList, ListOfConfigurableChilds, conf) {
      (*conf)->print(f, prefix, columns);
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


void Configurable::parse(FILE* f) {
  char* buffer = (char*)malloc(sizeof(char)*512);    
  assert(buffer);     
  while (fgets(buffer, 512, f) != 0) {
    if(strcmp(buffer,"######\n")==0) break;
    char* p = strchr(buffer,'=');
    if(p!=0){
      *p=0; // terminate string (key) at = sign	  
      char* s = buffer;
      while (*s==' ') s++; // skip leading spaces
      // cout << "Set: " << s << " Val:" << atof(p+1) << endl;	  
      setParam(s, atof(p+1)); 
    }    
  }
  free(buffer);
}

void Configurable::addConfigurable(Configurable* conf) {
  ListOfConfigurableChilds.push_back(conf);
  conf->parent=this;
}

void Configurable::removeConfigurable(Configurable* conf) {
  configurableList::iterator pos = std::posInColl<configurableList, const Configurable*>(ListOfConfigurableChilds, conf);
  if (pos != ListOfConfigurableChilds.end()) {
    conf->parent=0;
    ListOfConfigurableChilds.erase(pos);
  }
}

const Configurable::configurableList& Configurable::getConfigurables() const {
  return ListOfConfigurableChilds;
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
