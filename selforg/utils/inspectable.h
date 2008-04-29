/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
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
 *   Revision 1.2  2008-04-29 09:55:30  guettler
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
*/
class Inspectable {
public:

  /// TYPEDEFS BEGIN

  typedef std::string iparamkey;
  typedef double iparamval;
  typedef std::pair<iparamkey,iparamval*> iparampair;
  typedef std::pair<iparamkey,matrix::Matrix*> imatrixpair;

  typedef std::list<iparamkey> iparamkeylist;
  typedef std::list<iparamval> iparamvallist;
  typedef std::list<iparampair> iparampairlist;
  typedef std::list<imatrixpair> imatrixpairlist;


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

  /// TYPEDEFS END


  Inspectable();


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
     * @param key the name of the inspectable, shown e.g. in guilogger
     * @param val the address of the value to inspect
    */
  virtual void addInspectableValue(const iparamkey key, iparamval* val);


    /**
     * This is the new style for adding inspectable values. Just call this
     * function for each parameter and you are done.
     * inspects all elements of the given matrix given through the functions
     * store4x4AndDiagonalFieldNames(Matrix& m,string& matrixName);
     * defined in <selforg/controller_misc.h>
     * @param key the name of the matrix, shown e.g. in guilogger
     * @param m the address of the matrix to inspect
     * @note that you can change the structure of the matrix while
     * being inspected.
     */
  virtual void addInspectableMatrix(const iparamkey key, matrix::Matrix* m);



private:
  iparampairlist mapOfValues;
  imatrixpairlist mapOfMatrices;


};

#endif
