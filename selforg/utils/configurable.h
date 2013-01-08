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
#ifndef __CONFIGURABLE_H
#define __CONFIGURABLE_H

#include <iostream>
#include <cstdlib>

#include <limits>
#include <list>
#include <utility>
#include <string>
#include <map>
#include "stl_adds.h"
#include "backcaller.h"

/**
 Abstact class for configurable objects. Sort of Hashmap interface. Parameters are double, int or boolean values

 The Configurator is a (planned) external tool that can be used for changing the values of configurable objects.

 * Protocoll for Configurator:
 \code
 To Configurator (BNF notation):
 Conf         := <Comp>*
 Comp         := <CompName> <Pair>*
 CompName     := [<string>][<int>]<newline>
 Pair         := <alphanum>=<double><newline>
 \endcode
 the remaining tags are self explanatory

 Example
 \code
 [Component name which can contain spaces and digits and .,- ][ID1]
 key1 = value1
 key2 = value2
 .
 .
 [Other Component name which can contain spaces and digits and .,- ][ID2]
 key3 = value3
 \endcode

 Answer: (from Communicator to Simulation environment)\n
 1. On change of a parameter:
 \code
 [ID] key=newvalue
 \endcode
 or
 \code
 key=newvalue
 \endcode
 the latter one means that the parameter is changed in all components

 2. Request of the description as defined above.
 \code
 #Something I don\'t care
 \endcode
 */

class Configurable : public BackCaller
{
  public:

    typedef std::string paramkey;
    typedef std::string paramdescr;
    // params of type double
    typedef double paramval;
    typedef std::list<std::pair<paramkey, paramval> > paramlist;
    typedef std::map<paramkey, paramval*> parammap;

    // params of type bool
    typedef bool parambool;
    typedef std::list<std::pair<paramkey, parambool> > paramboollist;
    typedef std::map<paramkey, parambool*> paramboolmap;

    // params of type int
    typedef int paramint;
    typedef std::list<std::pair<paramkey, paramint> > paramintlist;
    typedef std::map<paramkey, paramint*> paramintmap;

    typedef std::map<paramkey, paramdescr> paramdescrmap;

    // stuff for bounds
    typedef std::pair<paramval, paramval> paramvalBounds;
    typedef std::map<paramkey, paramvalBounds> paramvalBoundsMap;
    #define valDefMaxBound 10.0
    #define valDefMinBound -10.0

    typedef std::pair<paramint, paramint> paramintBounds;
    typedef std::map<paramkey, paramintBounds> paramintBoundsMap;
    #define intDefMinBound -10
    #define intDefMaxBound 10

    typedef std::pair<paramkey, paramval*> paramvalpair;
    typedef std::pair<paramkey, parambool*> paramboolpair;
    typedef std::pair<paramkey, paramint*> paramintpair;

    typedef std::vector<Configurable*> configurableList;

    /// nice predicate function for finding by ID
    struct matchId : public std::unary_function<Configurable*, bool>
    {
        matchId(int id) :
          id(id)
        {
        }
        int id;
        bool operator()(Configurable* c)
        {
          return c->id == id;
        }
    };

    Configurable()
    {
      id = rand();
    }
    /// intialise with name and revision (use "$ID$")
    Configurable(const std::string& name, const std::string& revision) :
      name(name), revision(revision)
    {
      id = rand();
    }

//   Configurable(const Configurable& copy)
//     :id(copy.id), name(copy.name), revision(revision),
//      mapOfValues(mapOfValues), mapOfBoolean(mapOfBoolean), mapOfInteger(mapOfInteger)
//   {
//   }


    virtual ~Configurable()
    {
    }

    /**
       Is called when a parameter was changes via setParam(). Note that
       it is not called of parameters of childs are changed, then there notifyOnChange() method
       is called. The key and of the changed parameter
       (use getParam() to retrieve its actual value).
       Overload this function when special actions have to be taken on parameter changes.
    */
    virtual void notifyOnChange(const paramkey& key){}

    /**
     This is the new style for adding configurable parameters. Just call this function
     for each parameter and you are done.
     If you need to do some special treatment for setting (or getting) of the parameter
     you can handle this by overloading getParam and setParam
     */
    virtual void addParameter(const paramkey& key, paramval* val,
			      paramval minBound, paramval maxBound,
                              const paramdescr& descr = paramdescr() ) {
      mapOfValues[key] = val;
      if (minBound>*val) minBound = (*val)>0 ? 0 : (*val)*2;
      if (maxBound<*val) maxBound = (*val)>0 ? (*val)*2 : 0;
      if(!descr.empty()) mapOfDescr[key] = descr;
      mapOfValBounds[key]=paramvalBounds(minBound,maxBound);
    }

    ///  See addParameter(const paramkey& key, paramval* val, paramval minBound, paramval maxBound, const paramdescr& descr)
    virtual void addParameter(const paramkey& key, paramval* val,
			      const paramdescr& descr = paramdescr()){
      addParameter(key,val,valDefMinBound, valDefMaxBound, descr);
    }


    /**
     See addParameter(const paramkey& key, paramval* val) but for bool values
     */
    virtual void addParameter(const paramkey& key, parambool* val,
                            const paramdescr& descr = paramdescr()) {
      mapOfBoolean[key] = val;
      if(!descr.empty()) mapOfDescr[key] = descr;
    }

    /**
     See addParameter(const paramkey& key, paramval* val) but for int values
     */
    virtual void addParameter(const paramkey& key, paramint* val,
			      paramint minBound, paramint maxBound,
                              const paramdescr& descr = paramdescr()) {
      mapOfInteger[key] = val;
      if (minBound>*val) minBound = (*val)>0 ? 0 : (*val)*2;
      if (maxBound<*val) maxBound = (*val)>0 ? (*val)*2 : 0;
      if(!descr.empty()) mapOfDescr[key] = descr;
      mapOfIntBounds[key]=paramintBounds(minBound,maxBound);
    }

    virtual void addParameter(const paramkey& key, paramint* val,
			      const paramdescr& descr = paramdescr()){
      addParameter(key,val,intDefMinBound, intDefMaxBound, descr);
    }

    /**
     This function is only provided for convenience. It does the same as addParameter but set the
     variable to the default value
     */
    virtual void addParameterDef(const paramkey& key, paramval* val, paramval def,
				 paramval minBound, paramval maxBound,
                                 const paramdescr& descr = paramdescr()){
      *val = def;
      addParameter(key,val, minBound, maxBound, descr);
    }

    virtual void addParameterDef(const paramkey& key, paramval* val, paramval def,
				 const paramdescr& descr = paramdescr()){
      addParameterDef(key,val,def,valDefMinBound, valDefMaxBound, descr);
    }


    /// See addParameterDef(const paramkey&, paramval*, paramval)
    virtual void addParameterDef(const paramkey& key, parambool* val, parambool def,
                                 const paramdescr& descr = paramdescr()) {
      *val = def;
      addParameter(key,val,descr);
    }

    /// See addParameterDef(const paramkey&, paramval*, paramval)
    virtual void addParameterDef(const paramkey& key, paramint* val, paramint def,
				 paramint minBound, paramint maxBound,
                                 const paramdescr& descr = paramdescr()) {
      *val = def;
      addParameter(key,val, minBound, maxBound, descr);
    }

    virtual void addParameterDef(const paramkey& key, paramint* val, paramint def,
				 const paramdescr& descr = paramdescr()) {
      addParameterDef(key,val,def,intDefMinBound, intDefMaxBound, descr);
    }

    /// sets a description for the given parameter
    virtual void setParamDescr(const paramkey& key, const paramdescr& descr,
			       bool traverseChildren = true);


    /// return the id of the configurable objects, which is created by random on initialisation
    int getId() const {
      return id;
    }

    /// return the name of the object
    virtual paramkey getName() const {
      return name;
    }

    /// returns the revision of the object
    virtual paramkey getRevision() const {
      return revision;
    }

    /**
     *  Sets the name of the configurable.
     * @param name the name to set
     * @param callSetNameOfInspectable if true and if this instance is also inspectable,
     * call automatically setNameOfInspectable(name).
     */
    virtual void setName(const paramkey& name, bool callSetNameOfInspectable = true);

    /// sets the revision Hint: {  return "$ID$"; }
    virtual void setRevision(const paramkey& revision) {
      this->revision = revision;
    }

    /** returns the value of the requested parameter
     or 0 (+ error message to stderr) if unknown.
     */
    virtual paramval getParam(const paramkey& key, bool traverseChildren = true) const;

    /**
     * Returns if the requested parameter is part of the configurable or their children
     * @param key to search for
     * @return true if key found, otherwise false
     */
    virtual bool hasParam(const paramkey& key, bool traverseChildren = true) const;

    /** sets the value of the given parameter
     or does nothing if unknown.
     */
    virtual bool setParam(const paramkey& key, paramval val, bool traverseChildren = true);

    /**
     * Sets the bounds (minBound and maxBound) of the given parameter.
     * Not useful for parambool.
     * If minBound=maxBound, it is threated as that no bound is given.
     */
    virtual void setParamBounds(const paramkey& key, paramval minBound, paramval maxBound, bool traverseChildren = true);

    virtual void setParamBounds(const paramkey& key, paramint minBound, paramint maxBound, bool traverseChildren = true);

    virtual void setParamBounds(const paramkey& key, paramvalBounds bounds, bool traverseChildren = true);

    virtual void setParamBounds(const paramkey& key, paramintBounds bounds, bool traverseChildren = true);

    /** The list of all parameters with there value as allocated lists.
	Note that these are only parameters that are managed manually (with setParam, getParam)
	@see getAllParamNames()
     @return list of key-value pairs
     */
    virtual paramlist getParamList() const {
      return paramlist(); // return an empty list
    }

    /// returns all names that are configureable
    virtual std::list<paramkey> getAllParamNames(bool traverseChildren = true);

    virtual parammap getParamValMap() const {
      return mapOfValues;
    }

    virtual paramintmap getParamIntMap() const {
      return mapOfInteger;
    }

    virtual paramboolmap getParamBoolMap() const {
      return mapOfBoolean;
    }

    /// returns the description for the given parameter
    virtual paramdescr getParamDescr(const paramkey& key, bool traverseChildren = true) const;

    virtual paramvalBounds getParamvalBounds(const paramkey& key, bool traverseChildren = true) const;

    virtual paramintBounds getParamintBounds(const paramkey& key, bool traverseChildren = true) const;

    virtual bool hasParamDescr(const paramkey& key, bool traverseChildren = true) const;

    virtual bool hasParamvalBounds(const paramkey& key, bool traverseChildren = true) const;

    virtual bool hasParamintBounds(const paramkey& key, bool traverseChildren = true) const;

    /** stores the key values paires into the file : filenamestem.cfg
     including the comments given in the list
     */
    virtual bool storeCfg(const char* filenamestem, const std::list<std::string>& comments = std::list<std::string>());
    /** restores the key values paires from the file : filenamestem.cfg */
    virtual bool restoreCfg(const char* filenamestem);
    /// prints the keys, values and descriptions to the file. Each line is prefixed
    void print(FILE* f, const char* prefix, int columns=90, bool traverseChildren = true) const;

    /// parses the configuration from the given file
    bool parse(FILE* f, const char* prefix = 0, bool traverseChildren = true);

    /**
      * Adds a configurable as a child object.
      * @param conf the instance to add
      */
    virtual void addConfigurable(Configurable* conf);

    /**
      * Removes a configurable as a child object.
      * @param conf the instance to remove
      */
    virtual void removeConfigurable(Configurable* conf);

    /**
     * Returns the list containing all configurable children.
     */
    virtual const configurableList& getConfigurables() const;

    /**
     * Indicates that the configurable itself or the configurable children
     * attached to this configurable have changed.
     * This method must be called manually so that the Configurator GUI can react
     * at the changes.
     * This is done through the callbackable interface
     * (using CallbackableType CALLBACK_CONFIGURABLE_CHANGED).
     */
    virtual void configurableChanged();

    static const CallbackableType CALLBACK_CONFIGURABLE_CHANGED = 11;

  protected:
    /// copies the internal params of the given configurable
    void copyParameters(const Configurable&, bool traverseChildren = true);

    // internal function to print only description in multiline fasion
    void printdescr(FILE* f, const char* prefix, const paramkey& key,
                  int columns, int indent) const;


  private:
    int id;
    paramkey name;
    paramkey revision;

    parammap mapOfValues;
    paramboolmap mapOfBoolean;
    paramintmap mapOfInteger;
    paramdescrmap mapOfDescr;

    paramvalBoundsMap mapOfValBounds;
    paramintBoundsMap mapOfIntBounds;

    void initParamBounds(const paramkey& key);

    configurableList ListOfConfigurableChildren;
    Configurable* parent;
};

#endif // __CONFIGURABLE_H
