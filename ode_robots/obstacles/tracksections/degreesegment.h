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
 *   Revision 1.6  2005-12-07 22:19:11  robot3
 *   one tiny mistake fixed
 *
 *   Revision 1.5  2005/12/05 12:35:00  robot3
 *   showAABB is now configurable
 *
 *   Revision 1.4  2005/12/03 16:57:12  martius
 *   setWidth is void
 *
 *   Revision 1.3  2005/11/22 15:51:23  robot3
 *   testing
 *
 *   Revision 1.2  2005/11/22 13:03:31  robot3
 *   bugfixing
 *
 *   Revision 1.1  2005/11/15 14:29:25  robot3
 *   first version
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __DEGREESEGMENT_H
#define __DEGREESEGMENT_H

#include "abstracttracksection.h"

/**
 *  class for degree segments
 */
class DegreeSegment : public AbstractTrackSection {

 public:
  /**
   * Constructor
   */
  DegreeSegment();

  /**
   * Constructor
   */
  DegreeSegment(const Position& p,const double& angle);


  /**
   * Constructor
   */
  DegreeSegment(const Matrix& position);

  virtual ~DegreeSegment(){}


  /** 
   * sets the position and rotation(angle) of the segment
   * in the real world.
   */
void setPositionMatrix(const Matrix& pose);

  /** 
   * sets the position of the segment
   * in the real world.
   */
void setPosition(const Position& pos) ;

/**
 * returns the length of the segment,
 * here it is the length of the arc
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


virtual Position getPosition();

virtual void setRotation(const Matrix& m);

void setRotation(const double& angle);


virtual void setCurveAngle(const double& alpha);

virtual void setRadius(const double& rad);

virtual Matrix getRotation();


  /** 
   * gives the position and rotation(angle) of the segment
   * in the real world.
   * returns a matrix.
   */
  virtual Matrix getPositionMatrix();

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
bool isInside(const Position& p);


/**
 * returns a value between 0 and 100 that tells at which section
 * you are on the segment.
 * 0 means you are on the beginning
 * 100 means you are at the end
 * returns -1 if no IdValue can be given
 */
double getSectionIdValue(const Position& p);


/**
 * returns a value between 0 and 100 that tells at which width
 * you are on the segment, more to right or more to the left.
 * 0 means you are on the left
 * 50 means you are in the middle
 * 100 means you are on the right
 * returns -1 if no WidthValue can be given
 */
double getWidthIdValue(const Position& p);


/**
 * draws the obstacle (4 boxes for the playground)
 */
 void draw();



void create(dSpaceID space);


 void destroy();

 protected:
  // this is the radius of the curve
  double radius;
  // this is the width of the segment
  // normally it should be the same like alle the other segments
  double width;

  bool show_aabb;

  // the wall to be drawed
  list<dGeomID> innerWalls;
  list<dGeomID> outerWalls;

  double widthWall;
  double heightWall;

  // angle is for 90DegreeSegment is 90
  double angle;

  // determines if the curve goes left or right
  int left;

  bool obstacle_exists;

  /**
   * obstacle color
   */
  Color color;
 
  // stores the actual position and angle in a matrix
  Matrix posMatrix;

  void setProperties();

  /**
   * returns the inner coordinates that are responsible for the segment of the
   * given length and alpha. 
   */
  Position getInnerCoordinates(double length, double alpha);


  /**
   * returns the outer coordinates (world) that are responsible for the
   * segment of the given length and alpha. 
   */
  Position getOuterCoordinates(double length, double alpha);

};

#endif



