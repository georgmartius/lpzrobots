// -*- C++ -*-
/* 
   \file gnuplot.h
   simple C++ interface to gnuplot
*/

#ifndef GNUPLOT_H
#define GNUPLOT_H

#include <QString>

#include "plotinfo.h"

/** \defgroup baseclasses Base classes
 */
/// \ingroup baseclasses

/**
   Open a Gnuplot window and updates the last n values of channels defined
    in PlotInfo structure.
*/

class Gnuplot {
public: 
  Gnuplot() : plotInfo(0) {} 
  Gnuplot(const PlotInfo* plotinfo);
  
  ~Gnuplot();

  void init(const QString& gnuplotcmd, int w=400,int h=300, int x=-1, int y=-1);
  
  bool open(const QString& gnuplotcmd, int w=400,int h=300, int x=-1, int y=-1);

  void close();
  

  /** send arbitrary command to gnuplot.
      like "set zeroaxis" or other stuff */
  void command(const QString& cmd);


  /** make gnuplot plot channels */
  void plot();    

  /** creates the plot command
      if file is empty then the stdin is assumed ('-') and no using are given
   */
  QString plotCmd(const QString& file=QString(), int start=-1, int end=-1);
    
//   /** make gnuplot XY plot content of x against y data buffers 
//       use it as follow:
//       <pre>
//       T x[]={a,b,c};
//       T y[]={x,y,z};
//       plotXY(x,y,3);
//       </pre>
//   */
//   void plotXY(const T *x, const T *y,int size);    

//   /** make gnuplot XY plot content of x against y data buffers */
//   void plotXY(const T& x, const T& y);

private:
  const PlotInfo* plotInfo;
  FILE* pipe;
};

#endif

