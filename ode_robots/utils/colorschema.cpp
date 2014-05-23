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

#include "colorschema.h"
#include <osgDB/FileUtils>
#include <cassert>
#include <string.h>
#include <iostream>
#include <iomanip>


#include <selforg/stl_adds.h>

//Todo: add: allColors / printAllColors
//Todo: verbose load
//Todo: Aliases do not work

using namespace std;

namespace lpzrobots{

  ColorSchema::ColorSchema()
    : dummy(1.0,1.0,1.0,1.0) {

  }

  Color ColorSchema::color(const string& name_or_id_or_alias) const {
    return color(name_or_id_or_alias, 0);
  }

  Color ColorSchema::color(const std::string& name_or_id_or_alias, int alias_set) const {
    Color c;
    if(color(c, name_or_id_or_alias, alias_set)){
      return c;
    }else{
      return dummy;
    }
  }

  bool ColorSchema::color(Color& color, const string& name_or_id_or_alias,
                          int alias_set) const {
    if(getColor(color, name_or_id_or_alias)){
      return true;
    }else{
      AliasMap::const_iterator i = aliases.find(name_or_id_or_alias);
      if( i == aliases.end()){
        return false;
      }else{ // we know this alias
        const AliasVector& v = i->second;
        if((signed int)v.size() > alias_set && !v[alias_set].empty()){
          return getColor(color, v[alias_set]);
        }else{// don't have the alias in the set
          if(!v.empty())
            return getColor(color, v[0]); // try alias-set 0
          else
            return false;
        }
      }
    }
  }

  std::string ColorSchema::getLoadErrorString(int value) const {
    switch(value){
    case  0: return "No colors/aliases found";
    case -1:
      {
        osgDB::FilePathList l = osgDB::getDataFilePathList();
        join<string> p = for_each(l.begin(), l.end(), join<string>(","));
        return "Could not find file. Search pathes: " + p.joined;
      }
    case -2:
      return "Could not open file for reading";
    case -3:
      return "Parse error";
    case -4:
      return "Columns line not found or unsupported number (0,1)";
    default: return "no error";
    }
  }

  int ColorSchema::loadPalette(const string& gplfilename){
    string fname = osgDB::findDataFile(gplfilename, osgDB::CASE_INSENSITIVE);
    if(!fname.empty()){
      FILE* f = fopen(fname.c_str(),"r");
      if(!f) return -2;
      char s[1024];
      int columns;
      do{
        if(!fgets(s,1024,f)) return -3;
        if(strncmp(s,"Columns",7)==0)
          if(sscanf(s,"Columns: %i",&columns)!=1)
            return -4;
      }while(strncmp(s,"#",1)!=0);
      int r,g,b;
      int i=0;
      if(columns==0){
        while(fscanf(f,"%i %i %i %s\n",&r,&g,&b,s)==4){
          addColor(Color::rgb255(r,g,b), string(s));
          i++;
        }
      }else if(columns==1){
        char s2[1024];
        while(fscanf(f,"%i %i %i %s %s\n",&r,&g,&b,s,s2)==5){
          addColor(Color::rgb255(r,g,b), string(s));
          addColor(Color::rgb255(r,g,b), string(s2));
          i++;
        }
      }else{
        fprintf(stderr, "cannot read gpl file with %i name columns, support 0 or 1",
                columns);
        return -4;
      }
      fclose(f);
      return i;
    }else return -1;
  }


  int ColorSchema::loadAliases(const std::string& filename, int alias_set_offset){
    string fname = osgDB::findDataFile(filename, osgDB::CASE_INSENSITIVE);
    if(!fname.empty()){
      FILE* f = fopen(fname.c_str(),"r");
      if(!f) return -2;
      char alias[1024];
      char name[1024];
      int alias_set;
      int i=0;
      char s[1024];
      while(fgets(s,1024,f)) {
        if(s[0]=='#') continue;
        if(sscanf(s,"%s %s %i\n",alias,name,&alias_set)==3){
          if(addAlias(string(alias), string(name), alias_set+alias_set_offset)){
            i++;
          }
        }else if(sscanf(s,"%s %s\n",alias,name)==2){
          if(addAlias(string(alias), string(name), alias_set_offset)){
            i++;
          }
        }
      }
      fclose(f);
      return i;
    }else return -1;
  }


  void ColorSchema::addColor(const Color& color, const string& name){
    colors[name]=color;
  }

  bool ColorSchema::addAlias(const std::string& alias, const string& name,
                             int alias_set){
    assert(alias_set>=0);
    if(existsColor(alias)) {
      cerr << "cannot add alias " << alias << " because a color with that name exists\n";
      return false;
    }
    if(!existsColor(name)) {
      cerr << "cannot add alias " << alias << " to " << name
           << " because no color with that name exists\n";
      return false;
    }
    AliasMap::iterator i = aliases.find(alias);
    if( i == aliases.end()){
      AliasVector v;
      v.resize(alias_set+1);
      v[alias_set]=name;
      aliases[alias]=v;
    }else{ // we know this alias
      AliasVector& v = i->second;
      if((signed int)v.size() > alias_set){
        v[alias_set]=name;
      }else{
        v.resize(alias_set+1);
        v[alias_set]=name;
      }
    }
    return true;
  }

  void ColorSchema::setDefaultColor(const Color& c){
    dummy = c;
  }

  const Color& ColorSchema::getDefaultColor() const {
    return dummy;
  }

  bool ColorSchema::existsColor(const std::string& name) const {
    ColorMap::const_iterator i = colors.find(name);
    return (i != colors.end());
  }

  template<class T> struct print_func : public unary_function<T, void>
  {
    print_func(ostream& out, const string& delimit)
      : os(out), count(0), delimit(delimit) {}
    void operator() (T x) { os << x << delimit; ++count; }
    ostream& os;
    int count;
    string delimit;
  };


  void ColorSchema::print(ostream& out) const {
    out << "Colors:\n";
    FOREACHC(ColorMap, colors, c){
      out << setw(20) << c->first << ": " << c->second << endl;
    }
    out << "Aliases:\n";
    FOREACHC(AliasMap, aliases, a){
      const AliasVector& v = a->second;
      out << setw(20) << a->first << ": ";
      for_each(v.begin(), v.end(), print_func<string>(out, ",\t"));
      out << endl;
    }

  }

  bool ColorSchema::getColor(Color& c, const std::string& name) const {
    ColorMap::const_iterator i = colors.find(name);
    if(i != colors.end()){
      c=i->second;
      return true;
    }else
      return false;
  }


}
