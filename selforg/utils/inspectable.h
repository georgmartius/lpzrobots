/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    joergweide84@aol.com (robot12)                                       *
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
 *   $Log$
 *   Revision 1.14  2011-10-13 14:36:29  martius
 *   stl_adds removeElement
 *   zoo: adding and removing robots
 *
 *   Revision 1.13  2011/04/28 14:07:28  martius
 *   default name is empty
 *
 *   Revision 1.12  2011/03/22 11:01:53  guettler
 *   - added todo comments
 *
 *   Revision 1.11  2011/03/21 17:46:12  guettler
 *   enhanced inspectable interface:
 *   - support for inspectable children of a inspectable
 *   - some new helper functions
 *
 *   Revision 1.10  2011/01/11 13:17:27  guettler
 *   - added typedef for infoLinesList (necessary for FOREACH)
 *
 *   Revision 1.9  2010/10/18 15:08:28  martius
 *   matrices are now explicitly const (as it should be)
 *
 *   Revision 1.8  2009/10/14 09:58:29  martius
 *   added support for description strings that are exported using the infolines
 *
 *   Revision 1.7  2009/08/10 07:37:48  guettler
 *   -Inspectable interface now supports to add infoLines itself.
 *    These lines are then outprinted line by line to the PlotOption once,
 *    preceded by a #I.
 *   -Restart functionality of PlotOptionEngine added (e.g. closePipes(), reInit()).
 *
 *   Revision 1.6  2009/08/05 08:19:23  martius
 *   addInspectableMatrix allows to specify optionally whether all or only 4x4+Diagonal is used
 *
 *   Revision 1.5  2009/07/15 13:01:15  robot12
 *   one bugfixe
 *
 *   Revision 1.4  2009/06/29 13:24:13  robot12
 *   add function getInternalParamsPtr for new class inspectableproxy
 *
 *   Revision 1.3  2008/08/01 14:42:03  guettler
 *   we try the trip to hell! make selforg AVR compatible...good luck (first changes)
 *
 *   Revision 1.2  2008/04/29 09:55:30  guettler
 *   -class uses now a list of pairs instead of a map
 *   -debug printouts removed
 *
 *   Revision 1.1  2008/04/29 07:39:24  guettler
 *   -interfaces moved to selforg/utils
 *   -added addInspectableValue and addInspectableMatrix
 *   -methods getInternalParamNames and getInternalParams do not need to be
 *   overloaded anymore, use addInspectableValue and addInspectableMatrix
 *   instead (preferred)
 *
 *   Revision 1.9  2006/12/21 11:44:17  martius
 *   commenting style for doxygen //< -> ///<
 *   FOREACH and FOREACHC are macros for collection iteration
 *
 *   Revision 1.8  2006/07/20 17:14:34  martius
 *   removed std namespace from matrix.h
 *   storable interface
 *   abstract model and invertablemodel as superclasses for networks
 *
 *   Revision 1.7  2006/07/14 12:23:58  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.5.6.1  2006/06/25 16:51:35  martius
 *   configureable has name and revision
 *   a robot is configureable by default
 *
 *   Revision 1.5  2005/10/27 15:46:38  martius
 *   inspectable interface is expanded to structural information for network visualiser
 *
 *   Revision 1.4  2005/10/06 17:06:57  martius
 *   switched to stl lists
 *
 *   Revision 1.3  2005/09/11 11:20:01  martius
 *   virtual destructor
 *
 *   Revision 1.2  2005/08/06 20:47:54  martius
 *   Commented
 *
 *   Revision 1.1  2005/08/03 20:28:57  martius
 *   inspectable interface
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __INSPECTABLE_H
#define __INSPECTABLE_H


#include <list>
#include <map>
#include <utility>
#include <string>
#include "stl_adds.h"

#include <iostream>

namespace matrix {
  class Matrix;
}

/**
 * Interface for inspectable objects.
 * That means that one can read out some internal parameters indentified by string keys
 *
 * TODO: support for lead through of params (e.g. getInternalParams()) 
 * for all children inspectables. For this use a instance-wide switch (useChildren) 
 * as a member variable to enable this feature when desired.
*/
class Inspectable {
public:

  // TYPEDEFS BEGIN

  typedef std::string iparamkey;
  typedef double iparamval;
  typedef std::pair<iparamkey,iparamval*> iparampair;

  // the bool says whether  only 4x4AndDiag is used
  typedef std::pair<iparamkey,std::pair< const matrix::Matrix*, bool > > imatrixpair; 
  typedef std::list<iparamkey> iparamkeylist;
  typedef std::list<std::string> infoLinesList;
  typedef std::list<iparamval> iparamvallist;
  typedef std::list<iparamval*> iparamvalptrlist;
  typedef std::list<iparampair> iparampairlist;
  typedef std::list<imatrixpair> imatrixpairlist;

  // the network stucture export will be discontinued soon
  typedef struct ILayer{
    ILayer(const std::string& vectorname, const std::string& biasname,
	   int dimension, int rank, const std::string& layername)
      : vectorname(vectorname), biasname(biasname),
	 dimension(dimension), rank(rank), layername(layername) {}
    std::string vectorname;  //< prefix of the internal parameter vector e.g. "v"
    std::string biasname;    ///< prefix of the internal parameter vector used as bias for the neurons e.g. "h"
    int dimension;      ///< length of the vector (number of units)
    int rank;           ///< rank of the layer (0 are input layers)
    std::string layername;   ///< name of the layer as displayed by the visualiser
  }ILayer;

  typedef struct IConnection{
    IConnection(const std::string& matrixname, const std::string& vector1, const std::string& vector2)
      : matrixname(matrixname), vector1(vector1), vector2(vector2) {}
    std::string matrixname; ///< matrix name is the prefix of the internal parameter matrix e.g. "A"
    std::string vector1;    ///< vectorname of input layer
    std::string vector2;    ///< vectorname of output layer
  }IConnection;

  typedef std::list<ILayer> ilayerlist;
  typedef std::list<IConnection> iconnectionlist;

  /// nice predicate function for finding a Layer with its vectorname
  struct matchName : public std::unary_function<ILayer, bool> {
    matchName(std::string name) : name(name) {}
    std::string name;
    bool operator()(ILayer l) { return l.vectorname == name; }
  };

  typedef std::list<const Inspectable*> inspectableList;



  /// TYPEDEFS END


  Inspectable(const iparamkey& name = "");


  virtual ~Inspectable();


  /** The list of the names of all internal parameters given by getInternalParams().
      The naming convention is "v[i]" for vectors
       and "A[i][j]" for matrices, where i, j start at 0.
      @return: list of keys
   */
  virtual iparamkeylist getInternalParamNames() const;


/** @return: list of values
   */
  virtual iparamvallist getInternalParams() const;

  // should be protected, but it does not work
  /**
   * be careful: matrices will be ignored
   * @return list of values (pointer)
   */
  virtual iparamvalptrlist getInternalParamsPtr() const;

  /** Specifies which parameter vector forms a structural layer (in terms of a neural network)
      The ordering is important. The first entry is the input layer and so on.
      @return: list of layer names with dimension

   */
  virtual ilayerlist getStructuralLayers() const;


  /** Specifies which parameter matrix forms a connection between layers (in terms of a neural network)
      The orderning is not important.
      @return: list of layer names with dimension
   */
  virtual iconnectionlist getStructuralConnections() const;


  /**
   * This is the new style for adding inspectable values. Just call this
   * function for each parameter and you are done.
   * registers a single value
   * @param key the name of the inspectable, shown e.g. in guilogger (no spaces allowed)
   * @param val the address of the value to inspect
   * @param descr description string to be exported (using infolines)
   */
  virtual void addInspectableValue(const iparamkey& key, iparamval* val, 
                                   const std::string& descr = std::string());


  /**
   * This is the new style for adding inspectable values. Just call this
   * function for each parameter and you are done.
   * Inspects elements of the given matrix or vector (automatic detection)
   * For Matrixes Either all or only the values given by
   * store4x4AndDiagonalFieldNames(Matrix& m,string& matrixName);
   * are used.
   * @param key the name of the matrix, shown e.g. in guilogger (no spaces allowed)
   * @param m the address of the matrix to inspect
   * @param only4x4AndDiag of true only 4x4 matrix plus the diagonal is used 
   * @param descr description string to be exported 
   *              (for the entire matrix with key_) (using infolines)
   * @note that you can change the structure of the matrix while
   * being inspected, but no after the getInternalParameterNames is called!
   */
  virtual void addInspectableMatrix(const iparamkey& key, const matrix::Matrix* m, 
                                    bool only4x4AndDiag=true, 
                                    const std::string& descr = std::string());

  /** 
   * adds a description for the given parameter using info-lines
   * The line will start (appart from the #I) with a D for description
   * followed by the key end then followed by the string.
   * @param key parameter name (no spaces allowed)
   * @param descr descriptive string (no newlines allowed)
   */
  virtual void addInspectableDescription(const iparamkey& key, const std::string& descr);


  /**
   * Adds an info line to this inspectable instance. All infolines are plotted
   * by the PlotOptionEngine and therefore appear e.g. in the logfile.
   * They are leaded by a #I. If you have your own identifiers, just begin with
   * your one (e.g. N for number of
   * NOTE: You must not add e.g. carriage return, all this is handled by the
   * PlotOptionsEngine itself.
   * If you like to add multiple lines, call this function a lot more or
   * use instead addInfoLines.
   * @param infoLine the line (as string) to be added
   */
  virtual void addInfoLine(std::string infoLine);

  /**
   * Adds a bunch of infolines with addInfoLine to this inspectable instance.
   * Every infoline is separated by an carriage return automatically, done
   * by the PlotOptionsEngine.
   * @param infoLineList the infoLines to be added as a string list.
   */
  virtual void addInfoLines(std::list<std::string> infoLineList);

  /**
   * Removes all infolines from this inspectable instance.
   * This is useful if you have new infoLines and would like to restart
   * the PlotOptionEngine.
   */
  virtual void removeInfoLines();

  /**
   * Returns all infolines added to this inspectable instance.
   * This function is called by the PlotOptionEngine.
   * @return all added infolines.
   */
  virtual const infoLinesList& getInfoLines() const;

  /**
   * Adds an inspectable as a child object.
   * @param insp the instance to add
   */
  virtual void addInspectable(Inspectable* insp);

  /**
   * Removes an inspectable as a child object.
   * @param insp the instance to remove
   */
  virtual void removeInspectable(Inspectable* insp);


  /// set the name of the inspectable
  virtual void setNameOfInspectable(const iparamkey& name);

  /// return the name of the inspectable, getName() would conflict with Configurable::getName() too often
  virtual const iparamkey getNameOfInspectable() const;

  /**
   * Returns the list containing all inspectable children.
   */
  virtual const inspectableList& getInspectables() const;

  /* added when needed
  virtual void removeInspectable(const Inspectable* insp);

  virtual void removeAllInspectables();
  */

protected:
  iparamkey name;

  iparampairlist mapOfValues;
  imatrixpairlist mapOfMatrices;

  infoLinesList infoLineStringList;

private:
  inspectableList listOfInspectableChildren;
  bool printParentName;
  Inspectable* parent;


};

#endif
