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
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2009-03-27 06:16:57  guettler
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
#include <cstring>

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

void Configurable::print(FILE* f, const char* prefix) const {
  const char* pre = prefix==0 ? "": prefix;    
  const unsigned short spacelength=20;
  char spacer[spacelength];
  memset(spacer, ' ', spacelength);  spacer[spacelength-1]=0;
  // print header
  fprintf(f, "%s[%s, %s][%i]\n", pre, getName().c_str(), getRevision().c_str(), getId());
  // use map of values
  FOREACHC(parammap, mapOfValues, i) {
    const string& k = (*i).first;
    fprintf(f, "%s %s=%s%11.6f\n", pre, k.c_str(), 
	    spacer+(k.length() > spacelength  ? spacelength : k.length()), * (*i).second);
  }
  // add custom parameters stuff (which is marked by a * at the end of the line)
  paramlist list = getParamList(); 
  FOREACHC(paramlist, list, i) {
    const string& k = (*i).first;
    fprintf(f, "%s %s=%s%11.6f *\n", pre, k.c_str(), 
	    spacer+(k.length() > spacelength  ? spacelength : k.length()), (*i).second);
  }
  // write termination line
  fprintf(f, "######\n");
}

void Configurable::parse(FILE* f) {
  char* buffer = 0;    
  size_t len=0;    
  ssize_t read=0;    
  while ((read = getline(&buffer, &len, f)) != -1) {
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
  if(buffer) free(buffer);
}

#endif
