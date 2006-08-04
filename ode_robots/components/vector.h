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
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2006-08-04 14:49:15  martius
 *   matrices and vectors moved to compoments, which are depreciated
 *
 *   Revision 1.5  2006/07/14 12:23:57  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.4.4.1  2005/12/06 10:13:26  martius
 *   openscenegraph integration started
 *
 *   Revision 1.4  2005/11/09 13:31:51  martius
 *   GPL'ised
 *
 ***************************************************************************/
#ifndef vector_h
#define vector_h


#include <vector>
#include <math.h>
#include "exceptions.h"


namespace lpzrobots {


template<typename T>
class Vector {
 public:
  //typedef std::vector<T> Data;

 protected:
  std::vector<T> data;
  
 public:
  Vector(unsigned dimension = 0);

  void resize(unsigned new_dimension);

  unsigned get_dimension() const;
  T& operator() (unsigned i);
  const T& operator() (unsigned i) const;

  Vector<T> operator - (const Vector<T> &r_rhs) const;

  T length() const;
  T square_length() const;
};






template <typename T>
Vector<T>::Vector(unsigned dimension) :
  data(dimension)
{
}

template <typename T>
void Vector<T>::resize(unsigned new_dimension)
{
  data.resize(new_dimension);
}


template <typename T>
unsigned Vector<T>::get_dimension() const
{
  return data.size();
}


template <typename T>
T& Vector<T>::operator() (unsigned i)
{
  if(i >= data.size())
    IndexOutOfBoundsException().raise();


  return data[i];
}


template <typename T>
Vector<T> Vector<T>::operator - (const Vector<T> &r_rhs) const
{
  if(get_dimension() != r_rhs.get_dimension())
    IndexOutOfBoundsException().raise();
  
  unsigned n = get_dimension();

  Vector result(n);

  for(unsigned i = 0; i < n; ++i)
    result(i) = (*this)(i) - r_rhs(i);

  return result;
}

/*
template <typename T>
Vector<T> Vector<T>::operator + (const Vector<T> &r_rhs) const
{
  if(get_dimension() != r_rhs.get_dimension())
    IndexOutOfBoundsException().raise();
  
  unsigned n = get_dimension();

  Vector result(n);

  for(unsigned i = 0; i < n; ++i)
    result(i) = (*this)(i) + r_rhs(i);

  return result;
}
*/


template <typename T>
T Vector<T>::square_length() const
{
  unsigned n = get_dimension();

  T result = static_cast<T>(0);
  for(unsigned i = 0; i < n; ++i)
    result += (*this)(i) * (*this)(i);

  return result;
}


template <typename T>
T Vector<T>::length() const
{
  return sqrt(square_length());
}


template <typename T>
const T& Vector<T>::operator() (unsigned i) const
{
  if(i >= data.size())
    IndexOutOfBoundsException().raise();


  return data[i];
}







// note: Vector3 should be a partial specialization of Vector
template<class T> class Vector3 {
 protected:
  union {
    struct {
      T x;
      T y;
      T z;
    };
    T c[3]; // c = component
  };


 public:
  Vector3(const T &r_x = static_cast<T>(0),
          const T &r_y = static_cast<T>(0),
          const T &r_z = static_cast<T>(0)) :
    x(r_x),
    y(r_y),
    z(r_z)
  {
  }


  Vector3(const T* p)
  {
    memcpy(c, p, sizeof(c));
  }


  template <class U> Vector3(const Vector3<U> &r_other) :
    x(r_other.x),
    y(r_other.y),
    z(r_other.z)
  {
  }


  template <class U> Vector3<T> operator - (const Vector3<U> &r_rhs) const
  {
    return Vector3<T>(x - r_rhs.x,
		      y - r_rhs.y,
		      z - r_rhs.z);
  }


  template <class U> Vector3<T> operator + (const Vector3<U> &r_rhs) const
  {
    return Vector3<T>(x + r_rhs.x,
		      y + r_rhs.y,
		      z + r_rhs.z);
  }


  template <class U> Vector3<T> operator / (const U &r_rhs) const
  {
    return Vector3<T>(x / r_rhs,
                      y / r_rhs,
                      z / r_rhs);
  }



  Vector3<T> get_unit_vector() const
  {
    double l = length();
    if(0.0 == l)
      return *this;

    return Vector3<T>(x / l, y / l, z / l);
  }

    

  Vector3<T>& make_unit_length()
  {
    double l = length();
    if(0.0 == l)
      return *this;

    x /= l;
    y /= l;
    z /= l;
    
    return *this;
  }


  template <class U> Vector3<T>& operator = (const Vector3<U> &r_rhs)
  {
    x = r_rhs.x;
    y = r_rhs.y;
    z = r_rhs.z;

    return *this;
  }


  /*
  const Vector3& operator = (const Vector3 &r_rhs) {
    x = r_rhs.x;
    y = r_rhs.y;
    z = r_rhs.z;

    return *this;
  }
  */

  double square_length() const
  {
    return static_cast<double>(x*x + y*y + z*z);
  }


  double length() const
  {
    return sqrt(square_length());
  }

  operator T* () {
    return &c[0];
  }
};


typedef Vector3<double> DVector3;
typedef Vector3<float>  FVector3;


}



#endif
