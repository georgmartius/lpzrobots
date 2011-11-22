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
#ifndef __COLORSCHEMA_H
#define __COLORSCHEMA_H

#include "color.h"
#include <selforg/stl_map.h>
#include <string>
#include <vector>
#include <iostream>

namespace lpzrobots{

  /**
     A store for colors with a set of aliases.
     The alias-sets are numbered where the 0'th plays the role of a default set.
   */
  class ColorSchema 
  {
  public:
    typedef HashMap<std::string, Color> ColorMap;
    typedef std::vector<std::string> AliasVector;
    typedef HashMap<std::string, AliasVector > AliasMap;    

    ColorSchema();
  
    /** retrieves a color with the given name/id/alias
        if no color is found that matches the id/alias then 
        the default color is returned.
        Always the alias-set 0 is checked first
    */
    Color color(const std::string& name_or_id_or_alias) const;

    /** retrieves a color with the given name/id/alias from given alias_set
        if not found then the default alias_set (0) is checked
     */
    Color color(const std::string& name_or_id_or_alias, int alias_set) const;

    /** call by reference version      
        returns false if color not found
    */
    bool color(Color& color, const std::string& name_or_id_or_alias, 
               int alias_set = 0) const;

    /// checks whether color with the name exists (no aliases are checked)
    bool existsColor(const std::string& name) const;

    /** loads a gpl (gimp pallette file) and returns the number loaded colors
        The name of the colors should not contain white spaces!
     */    
    int loadPalette(const std::string& gplfilename);
    /** loads aliases from text file with lines containing:\n
        aliasname colorname/id [alias-set]
        @param alias_set_offset number that is added to the alias_set number in the file
     */
    int loadAliases(const std::string& filename, int alias_set_offset = 0);

    /** adds a color to the color store 
        (to add the id call the function twice with id as name)
    */
    void addColor(const Color& color, const std::string& name);

    /** adds a color alias (into the given alias-set)
        @param name name/id of existing color 
        @param alias new name 
        @return true if alias was stored or
         false if color name does not exists or 
               alias names a color and is therefor rejected
    */
    bool addAlias(const std::string& name, const std::string& alias, int alias_set = 0);

    void setDefaultColor(const Color& c);
    const Color& getDefaultColor() const;

    /// returns error string for value returned by loadPalette and loadAliases
    std::string getLoadErrorString(int value) const;
    
    /// prints all colors and aliases
    void print(std::ostream& out) const ;

  protected:
    // only name/id no alias checking
    bool getColor(Color&, const std::string& name) const;

  private:
    Color dummy;
    ColorMap colors;
    AliasMap aliases;    
  };
}

#endif
