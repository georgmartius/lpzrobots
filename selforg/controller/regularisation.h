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
#ifndef __REGULARISATION_H
#define __REGULARISATION_H

#include <cmath>
#include <selforg/controller_misc.h>

double inline sqr(double x) { 
  return x*x; 
}

/// neuron transfer function
double inline g(double z)
{
  return tanh(z);
};

/// first dervative
double inline g_s(double z)
{
  double k=tanh(z); 
  return 1.025 - k*k; 
  //  return 1/((1+0.5 * z*z)*(1+0.5 * z*z));    // softer
  //return 1/(1+log(1+z*z)); // even softer
};


/// first dervative with smoothing for large z
double inline g_derivative(double z)
{  
   return 1/((1+0.5 * z*z)*(1+0.5 * z*z));  
};

/// inverse of the first derivative
double inline g_s_inv(double z)
{
  double k=tanh(z);
   return 1/(1.025 - k*k);
  // return 1+z*z; // softer
  //return 1+log(1+z*z); // even softer
};

/** \f[ g'(z+xsi) = 1-(tanh(z+xsi))^2 \f] with additional clipping */
double inline g_s(double z, double xsi) {
  double Z = clip(z, -3.0, 3.0) + clip(xsi, -1.0, 1.0);  
  double k=tanh(Z);  // approximation with Mittelwertsatz 
  return 1 - k*k;
};


/** soft version: \f[ g'(z+xsi) = 1/(1+(z+xsi)^2 \f] with additional clipping */
double inline g_s_soft(double z, double xsi) {
  double Z = clip(z, -3.0, 3.0) + clip(xsi, -1.0, 1.0);  
  return 1/(1 + Z*Z);//TEST
};


/// an exact formula for g''/g'= -2g(Z), with clipped Z = z+xsi
double inline g_ss_div_s(double z, double xsi) { 
  // for consistency reasons we use the same clipped z as for g'.
  double Z = clip(z, -3.0, 3.0) + clip(xsi, -1.0, 1.0);  
  // approximation with Mittelwertsatz (z is clipped)
  return -2*g(Z);
};

/// an soft formula for g''/g' = -2Z, with clipped Z = z+xsi
double inline g_ss_div_s_soft(double z, double xsi) { 
  // for consistency reasons we use the same clipped z as for g'.
  double Z = clip(z, -3.0, 3.0) + clip(xsi, -1.0, 1.0);  
  return -2*Z;//TEST
};

/** with \f[ g'(z) = 1-(g(z+\xi))^2 \f] we get 
    \f[\frac{\partial}{\partial z} \frac{1}{g'(Z)} = \frac{g''}{g'^2} \f] 
    again with clipped Z
 */
double inline derive_g_s_inv_exact_clip(double z, double xsi){
  double Z = clip(z, -3.0, 3.0) + clip(xsi, -1.0, 1.0);  
  double k=tanh(Z);  // approximation with Mittelwertsatz 
  return -2*k/(1-k*k);
}

/** \f[ g'(z) = 1-(z+\xi)^2 \f] which is the series expansion to the second order
*/ 
double inline g_s_expand2(double z, double xsi){
  double Z = z + clip(xsi, -fabs(z), fabs(z));  
  return 1/(1+sqr(Z));
}

/** \f[ \frac{1}{g'(z)} \approx 1+(z+\xi)^2 \f] with geometric series approximation
*/ 
double inline g_s_inv_expand2(double z, double xsi){
  double Z = z + clip(xsi, -fabs(z)/2.0, fabs(z)/2.0);  
  return 1+sqr(Z);
}

/** \f[ \frac{g''(z)}{g'(z)} \approx 2(z+\xi)(1+(z+\xi)^2) \f] with geometric series approximation
*/ 
double inline g_ss_div_s_expand2(double z, double xsi){
  double Z = z + clip(xsi, -fabs(z)/2.0, fabs(z)/2.0);  
  //  double Z = z + clip(xsi, -fabs(z), fabs(z));  
  return -2*tanh(Z); 
}


  /// squashing function (-0.1 to 0.1), to protect against to large weight updates
double inline squash(double z)
  {
    return clip(z, -0.1, 0.1);
    //return 0.1 * tanh(10.0 * z);
  };
 
  /// squashing function with adjustable clipping size, to protect against too large weight updates
double inline squash(void* d, double z) {
    double size = *((double*)d);
    return clip(z, -size, size);
  };



#endif

