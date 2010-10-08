/*
 * VectorPlotVisualisation.h
 *
 *  Created on: 29.04.2010
 *      Author: oni
 */

#ifndef VECTORPLOTVISUALISATION_H_
#define VECTORPLOTVISUALISATION_H_
#include "AbstractVisualisation.h"
#include <qgl.h> 

class VectorPlotVisualisation: public AbstractVisualisation {
  Q_OBJECT

public:
  VectorPlotVisualisation(MatrixPlotChannel *channel, ColorPalette *colorPalette, QWidget *parent = 0);
//  VectorPlotVisualisation(VectorPlotChannel *channel, ColorPalette *colorPalette, QWidget *parent = 0);
  virtual ~VectorPlotVisualisation();
  //void updateView();


protected:
  void initializeGL();
  void resizeGL(int w, int h);
  void paintGL();
  void mouseMoveEvent ( QMouseEvent *event );


private:
  int maxX, maxY;
  const static bool debug = false;

};

#endif /* VECTORPLOTVISUALISATION_H_ */
