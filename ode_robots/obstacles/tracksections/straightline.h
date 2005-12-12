/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *   Revision 1.4  2005-12-12 13:44:41  martius
 *   barcodesensor is working
 *
 *   Revision 1.3  2005/12/03 16:57:12  martius
 *   setWidth is void
 *
 *   Revision 1.2  2005/11/22 15:51:23  robot3
 *   testing
 *
 *   Revision 1.1  2005/11/15 14:29:25  robot3
 *   first version
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __STRAIGHTLINE_H
#define __STRAIGHTLINE_H

#include "abstracttracksection.h"

/**
 *  Abstract class (interface) for obstacles
 */
class StraightLine : public AbstractTrackSection {

public:

  /**
   * Constructor
   */
  StraightLine(const Position& p,const double& angle);

  /**
   * Constructor
   */
  StraightLine(const Matrix& pose);

  virtual ~StraightLine(){}

  void setCurveAngle(double alpha);

  /**
   * gives the position and rotation(angle) of the segment at the
   * end of the segment so that a new segment could be placed there
   * if you want to place the new segment, you must muliplicate:
   * getTransformedEndMatrix()*getPositionMatrix();
   */
  virtual Matrix getTransformedEndMatrix();


  /**
   * returns true if the real coordinates lay inside of the segment
   */
  virtual bool isInside(const Position& p);


  virtual double getSectionIdValue(const Position& p);

  virtual double getWidthIdValue(const Position& p);


  /**
   * returns the length of the segment,
   * here it is the length of the arc
   * formula is: radius * angle;
   */
  virtual double getLength();
  
  /**
   * returns the width of the segment,
   */
  virtual double getWidth();
  
  /**
   * sets the width of the segment,
   */
  virtual void setWidth(double w);
  
  
  /**
   * draws the obstacle (4 boxes for the playground)
   */
  virtual void draw();  
  
  virtual void create(dSpaceID space);
  
  virtual void destroy();
  
protected:
  // this is the length of the segment
  double length;
  // this is the width of the segment
  // normally it should be the same like alle the other segments
  double width;
  dGeomID wallLeft; // the wall left to the street
  dGeomID wallRight; // the wall right to the street
  double widthWall;
  double heightWall;
  // angle is for straightline 0
  double angle;
  // determines if the curve goes right or left
  double isLeft;

  bool obstacle_exists;

  /**
   * obstacle color
   */
  Color color;
 
  void setProperties();
};

#endif
