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
 *   Revision 1.1  2005-11-15 14:27:21  robot3
 *   first version
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __RACEGROUND_H
#define __RACEGROUND_H


#include <stdio.h>
#include <math.h>
#include <list>

#include "abstractobstacle.h"
#include <drawstuff/drawstuff.h>
#include "stl_adds.h"
#include "matrix.h"
using namespace matrix;
#include "mathutils.h"
#include "abstracttracksection.h"
#include "straightline.h"
#include "degreesegment.h"


//Fixme: raceground creates collisions with ground and itself
class Raceground : public AbstractObstacle {

  // the list that contains all segments
  list<AbstractTrackSection*> SegmentList;


  bool obstacle_exists;

 public:
  
  Raceground(const OdeHandle& odehandle, double factorxy = 1):
    AbstractObstacle(odehandle) {
    obstacle_exists=false;
    setColor(226 / 255.0, 103 / 255.0, 66 / 255.0);
  };

  /**
   * adds the segments in the list to the SegmentList
   */
  void addSegments(list<AbstractTrackSection*> listToAdd) {
    SegmentList+=listToAdd;
  }

  /**
   * adds the segment to the SegmentList
   */
  void addSegment(AbstractTrackSection* Segment) {
    SegmentList+=Segment;
  }


  /**
   * adds the named segment to the SegmentList
   * names are:
   * straightline: StraightLine
   * 90degree    : DegreeSegment
   */
  void addSegment(string& name) {
    // get first pose from last stored segment
    Matrix newPose = ::getTranslationRotationMatrix(Position(0.0f,0.0f,0.0f),0.0f*M_PI/6.0f);
    if (!SegmentList.empty()) {
      Matrix pos = SegmentList.back()->getPositionMatrix();
      assert (pos.getM()==4 && pos.getN()==4);
      Matrix end = SegmentList.back()->getTransformedEndMatrix();
      assert (end.getM()==4 && end.getN()==4);
      newPose=pos * end;
    }
    //TODO: write a parser for the parameters
    if (name=="straightline") {
      std::cout << newPose;
      AbstractTrackSection* segment = new StraightLine(newPose);
      SegmentList += segment;
    } else if (name.compare("degree")<0) {
      std::cout << newPose;
      DegreeSegment* segment = new DegreeSegment(newPose);
      SegmentList+=(AbstractTrackSection*) segment;
      // now get the angle and the radius
      char* d;
      double angle, radius;
      if (sscanf(name.c_str(),"%s %lf %lf",d,&angle,&radius)!=3)
	std::cout << "parameter parsing invalid!: " << name << "\n";
      else {
	// parsing was ok
	segment->setCurveAngle(angle);
	segment->setRadius(radius);
      }
    }
  }

  /**
   * adds the named segments in the list to the SegmentList
   * names are:
   * straightline: StraightLine
   * 90degree    : 90DegreeSegment
   */
  void addSegments(list<string> names) {
    for(list<string>::iterator it = names.begin(); it!=names.end(); ++it) {
      addSegment(*it);
    }
  }

  /**
   * draws all the segments stored in SegmentList
   */
  virtual void draw(){
    //    double box[3];
    dsSetTexture (DS_NONE);    
    dsSetColor (color.r, color.g, color.b);
    for(list<AbstractTrackSection*>::iterator it = SegmentList.begin();
							it!= SegmentList.end(); ++it) {
      // call the create function of the segment
      (*it)->draw();
    }
 };
  
  
  virtual void setPosition(double x, double y, double z){
    if (obstacle_exists){
      destroy();
    }
    create();
  };

  virtual void getPosition(double& x, double& y, double& z){
    x=0.0f;
    y=0.0f;
    z=0.0f;
  };
  

  // normally we don't need this function
  virtual void setGeometry(double length_, double width_, double height_){
    length=length_;
    width=width_;
    height =height_;
  };

  virtual void setColor(double r, double g, double b){
    color.r=r;
    color.g=g;
    color.b=b;
  };

 protected:
  double length;
  double width;
  double height;

  dSpaceID raceground_space;

  virtual void create(){
    // create raceground space and add it to the top level space
    raceground_space = dSimpleSpaceCreate (space);
    dSpaceSetCleanup (raceground_space,0);
    // todo:
    // go through all list elements
    // draw them all by there own draw function
    for(list<AbstractTrackSection*>::iterator it = SegmentList.begin();it!= SegmentList.end(); ++it) {
      // call the create function of the segment
      (*it)->create(raceground_space);
    }
  };


  virtual void destroy(){
    for(list<AbstractTrackSection*>::iterator it = SegmentList.begin();it!= SegmentList.end(); ++it) {
      // call the create function of the segment
      (*it)->destroy();
    }
    obstacle_exists=false;
  };

};

#endif
