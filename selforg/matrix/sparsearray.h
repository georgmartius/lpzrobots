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
#ifndef __SPARSEARRAY_H_
#define __SPARSEARRAY_H_

#include <cstdlib>
#include "stl_map.h"

/**
 * Array which uses an HashTable for elements stored in this array.
 * Can be used as an base class of SparseMatrix.
 * first (fast implemented) version.
 */
namespace matrix
{

  #define COMPARE_EPS 1e-12

  // forward declaration
  //template<typename I, typename D> class SparseArray;



  template<typename I, typename D> class SparseArray
  {
  public:
    class ArrayElement
    {
    public:
      ArrayElement() : index(0), value(0), hashData(0), dummy(0) { }

      inline void reallocate(HashMap<I, D* >* hashData)
      {
        this->hashData=hashData;
        value=0;
        dummy=0;
      }

      inline void set (const I index, D value, D* dummy)
      {
        this->index=index;
        this->value=value;
        this->dummy=dummy;
      }

      // TODO: could be tricky: two ArrayElements at the same time???
      inline D operator+ (ArrayElement& el2)            { return value+el2.value; }
      inline D operator+ (D el2)                        { return value+el2; }
   /*   inline D operator++ ()                            { this=(value+1); return value; }*/
      inline bool operator> (D el2)                     { return value>el2; }
      // TODO: could be tricky: two ArrayElements at the same time???
      inline bool operator> (ArrayElement& el2)         { return value>el2.value; }
      inline D& operator= (D value)
      {
        if (dummy==0)
        {
          dummy = (D*) malloc(sizeof(D));
          (*hashData)[index]=dummy;
        }
        *dummy = value;
        return *dummy;
      }
      inline D& operator= (ArrayElement& el2)           { this=el2.value; }

      inline operator D()                               { return value; }

    protected:
      I index;
      D value;
      HashMap<I, D* >* hashData;
      D* dummy;
    };

    SparseArray(I arraySize) : arraySize(arraySize), hashData(0)
    {
      allocate();
    }

    virtual ~SparseArray() {
      freeData();
    }

    virtual inline void reallocate(I arraySize)
    {
      this->arraySize=arraySize;
      allocate();
    }

    virtual inline I size() { return this->arraySize; }

    inline ArrayElement& operator[](const I index)
    {
      typename HashMap<I,D*>::const_iterator iterator = hashData->find(index);
      if (iterator!= hashData->end())
      {
        D* dummy = (*iterator).second;
        elementDummy.set(index, *dummy, dummy);
      } else
        elementDummy.set(index, 0, 0);
      return elementDummy;
    }

    /** sets the data (row-wise).
        @param _data if null then matrix elements are set to zero
        otherwise the field MUST have the length should be getSize()*/
/*    virtual void inline set(const D* _data)
    {
      allocate();
      for (I i=0;i<arraySize;i++)
        if (_data[i]>COMPARE_EPS)
          this[i]=_data[i];
    }*/

    virtual inline long getRealSize()
    {
      long sumSize = sizeof *this;
      sumSize += sizeof *hashData;
      sumSize += hashData->size() * (sizeof(D*)+sizeof(D));
      return sumSize;
    }


  protected:
    ArrayElement elementDummy;
    /// set of all array values
    I arraySize;
    HashMap<I, D* >* hashData;

    virtual void inline allocate()
    {
      if (hashData)
        freeData();
      hashData = new HashMap<I, D* >();
      elementDummy.reallocate(hashData);
    }

    virtual void inline freeData()
    {
      // TODO: free all elements in hashData
//      for (HashMap<I,D*>::iterator iterator = hashData->begin();iterator!=hashData->end();iterator++)
//        free((*iterator).second);
      if (hashData)
      {
        delete hashData;
        hashData = 0;
      }
    }

  private:
    SparseArray() {}

  };


} // namespace matrix

#endif /* __SPARSEARRAY_H_ */
