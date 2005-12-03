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
 *   Revision 1.7  2005-12-03 16:56:28  martius
 *   removed own space because of weird no-collision bug
 *
 *   Revision 1.6  2005/11/29 13:38:28  robot3
 *   raceground can now be placed by constructor
 *
 *   Revision 1.5  2005/11/22 15:49:22  robot3
 *   bugfixing
 *
 *   Revision 1.4  2005/11/22 13:01:41  robot3
 *   tiny improvements
 *
 *   Revision 1.3  2005/11/22 10:22:14  martius
 *   changed to capital G in Ground
 *
 *   Revision 1.2  2005/11/15 14:50:52  martius
 *   correct parsing of "degree"
 *
 *   Revision 1.1  2005/11/15 14:27:21  robot3
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
class RaceGround : public AbstractObstacle {

  // the list that contains all segments
  list<AbstractTrackSection*> SegmentList;


  bool obstacle_exists;

 public:

  RaceGround(const OdeHandle& odehandle):
    AbstractObstacle(odehandle) {
    setParameters(pose);
  };

  
  RaceGround(const OdeHandle& odehandle,const Matrix& pose):
    AbstractObstacle(odehandle) {
    setParameters(pose);
  };

  RaceGround(const OdeHandle& odehandle,const Position& pos, double angle):
    AbstractObstacle(odehandle) {
    setParameters(getTranslationRotationMatrix(pos,angle));
 };


  /**
   * Destructor
   */
  ~RaceGround() {}

  /**
   * you set the number of segments of the track
   */
  void setNumberOfSegments(int number) {
    numberOfSegments=number;
  }


  /**
   * returns the segment number of the given point
   * returns -1 if point is not on the track
   */
  double getWidthOfRobot(const Position& p) {
    // todo: ask all segments if the robot is one of them and give back
    // the width
    for(list<AbstractTrackSection*>::iterator it = SegmentList.begin();
	it!= SegmentList.end(); ++it) {
      double widthID = (*it)->getWidthIdValue(p); 
      if (widthID>=0.0f) { // segment is found
	return widthID;
      }
    }
    return -1.0f;
  }



  /**
   * returns the segment number of the given point
   * returns -1 if point is not on the track
   */
  double getSegmentNumberOfRobot(const Position& p) {
    // todo: ask all segments if the robot is one of them and give back
    // the segmentID
    double passedLength=0.0f;
    double segmentNumber=-1.0f;
   for(list<AbstractTrackSection*>::iterator it = SegmentList.begin();
	it!= SegmentList.end(); ++it) {
      double sectionLength = (*it)->getSectionIdValue(p); 
      if (sectionLength>=0.0f) { // segment is found
	sectionLength*=((*it)->getLength())/100.0f;
	segmentNumber=numberOfSegments*
	  (passedLength+sectionLength)/trackLength;
	return segmentNumber;
      }
      /*      cout << segmentNumber << "=segMentNumber\n";
      cout << passedLength << "=passedLength\n";
      cout << sectionLength << "=sectionLength\n";
      cout << (*it)->getLength() << "given segment length!";*/
      passedLength+=(*it)->getLength();
      /*      cout << passedLength << "=passedLength!!!!!!!\n";
	      cout << nr++ << ". durchlauf\n";*/
    }
    return segmentNumber;
  }

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
    Matrix newPose(pose); // this is the initial pose
    if (!SegmentList.empty()) {
      Matrix pos = SegmentList.back()->getPositionMatrix();
      Matrix end = SegmentList.back()->getTransformedEndMatrix();
      newPose=pos * end;
    }
    //TODO: write a parser for the parameters (done)
    if (name=="straightline") {
      AbstractTrackSection* segment = new StraightLine(newPose);
      SegmentList += segment;
      trackLength+=segment->getLength();
    } else if (name.find("degree")==0) {
      DegreeSegment* segment = new DegreeSegment(newPose);
      SegmentList+=(AbstractTrackSection*) segment;
      // now get the angle and the radius
      char* d = (char*)malloc(name.length()*sizeof(char));
      double angle, radius;
      if (sscanf(name.c_str(),"%s %lf %lf",d,&angle,&radius)!=3)
	std::cout << "parameter parsing invalid!: " << name << "\n";
      else {
	std::cout << "parameters " << d << "," << angle << " " << radius << "\n";  
	// parsing was ok
	segment->setCurveAngle(angle/180.0*M_PI);
	segment->setRadius(radius);
        trackLength+=segment->getLength();
      }
      free(d);
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
  double trackLength; 
  double width; 
  double height;
  Matrix pose;
  double numberOfSegments;
  
  dSpaceID raceground_space;
  
  virtual void setParameters(Matrix initpose) {
    pose=initpose;
    obstacle_exists=false;
    setColor(226 / 255.0, 103 / 255.0, 66 / 255.0);
    numberOfSegments=256.0f;
    trackLength=0.0;
  }

  virtual void create(){
    // create raceground space and add it to the top level space
    raceground_space = space;
	//raceground_space = dSimpleSpaceCreate (space);
    //     dSpaceSetCleanup (raceground_space,0);
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
