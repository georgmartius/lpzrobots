#ifndef vector_h
#define vector_h


#include <vector>
#include <math.h>
#include "exceptions.h"


namespace university_of_leipzig {
namespace robots {


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
}


#endif
