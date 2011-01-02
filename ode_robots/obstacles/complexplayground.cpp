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
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.7  2011-01-02 23:09:52  martius
 *   texture handling of boxes changed
 *   playground walls changed
 *
 *   Revision 1.6  2010/04/27 14:40:02  martius
 *   depth==0 is ignored for drawing but used for bounding box
 *
 *   Revision 1.5  2009/03/13 09:19:53  martius
 *   changed texture handling in osgprimitive
 *   new OsgBoxTex that supports custom texture repeats and so on
 *   Box uses osgBoxTex now. We also need osgSphereTex and so on.
 *   setTexture has to be called before init() of the primitive
 *
 *   Revision 1.4  2008/05/07 16:45:51  martius
 *   code cosmetics and documentation
 *
 *   Revision 1.3  2008/04/23 07:17:16  martius
 *   makefiles cleaned
 *   new also true realtime factor displayed,
 *    warning if out of sync
 *   drawinterval in full speed is 10 frames, independent of the speed
 *
 *   Revision 1.2  2007/09/06 18:46:41  martius
 *   printContours
 *
 *   Revision 1.1  2007/08/28 09:23:20  martius
 *   initial
 *
 *
 *                                                                 *
 ***************************************************************************/

#include "complexplayground.h"
#include "primitive.h"
#include "osgprimitive.h"
#include <selforg/stl_adds.h>
#include <selforg/matrix.h>
#include <selforg/controller_misc.h>

namespace lpzrobots {

  using namespace std;
  using namespace matrix;

  int PolyLine::parse(std::list<char*> lines){
    int pen_color, fill_color;
    list<char*>::iterator l = lines.begin();
    if(lines.size()<2) return 1;    
    int r = sscanf(*l,"%i %i %i %i %i %i %i", &object_code, &sub_type,&line_style, &thickness, 
		   &pen_color, &fill_color, &depth);
    int i=0;
    int dat[2];
    if(r==7 && object_code==2){
      l++;
      int linecnt=1;
      while(l!=lines.end()){
	char* line = *l;
	if(line[0]=='\t'){
	  char* p;
	  p=strtok(line+1," ");
	  if(!p) return false;
	  dat[i] = atoi(p);    
	  i++;
	  while((p=strtok(NULL," "))!=NULL )  {
	    dat[i] = atoi(p);
	    i++;
	    if(i==2){
	      points.push_back(Pos(dat[0]/450.0,-dat[1]/450.0,0));
	      i=0;
	    }
	  };
	}else break;
	l++;
	linecnt++;
      }
      return linecnt;
    }else return 1;    
  }

  void PolyLine::print(){
    printf("%i %i %i %i\n\t", object_code, sub_type, line_style, thickness);
    FOREACH(list<Pos>, points, p){
      printf("%f %f", p->x(), p->y() );
    }
    printf("\n");
  }



  
  ComplexPlayground::ComplexPlayground(const OdeHandle& odeHandle, 
				       const OsgHandle& osgHandle , 
				       const std::string filename,
				       double factor, double heightfactor, bool createGround)
    : AbstractGround(odeHandle, osgHandle, createGround, 1,1,0.1), 
      filename(filename), factor(factor), heightfactor(heightfactor) {
    
    FILE* f=fopen(filename.c_str(),"r");
    if(!f){
      fprintf(stderr,"Cannot open file: %s!\n",filename.c_str());
      exit(1);
    }
    list<char*> lines;    
    char* buffer = (char*)malloc(sizeof(char)*4096);
    while(fgets(buffer, 4096, f)){
      lines.push_back(buffer);
      buffer = (char*)malloc(sizeof(char)*4096);
    }
    while(lines.size()>0){
      PolyLine p;     
      int consumed = p.parse(lines);
      if(consumed>1){	
	polylines.push_back(p);	
      }
      for(int i=0; i<consumed; i++){
	free(*lines.begin());
	lines.pop_front();
      }
    }
//     FOREACH(list<PolyLine>, polylines, p){
//       p->print();
//     }
  };


  void ComplexPlayground::create(){
    PolyLine boundary;
    bool hasboundary=false;
    FOREACH(list<PolyLine>, polylines, p){
      if(p->line_style==1){
	boundary = *p;
	hasboundary=true;
	break;
      }
    }
    if(hasboundary){
      int l = boundary.points.size();
      Matrix xs(l,1);
      Matrix ys(l,1);
      int i=0;
      FOREACH(list<Pos>, boundary.points, p){
	xs.val(i,0)=p->x();
	ys.val(i,0)=p->y();
      }
      double xsize = max(xs.map(fabs));
      double ysize = max(ys.map(fabs));
      groundLength = 2*xsize*factor;
      groundWidth  = 2*ysize*factor;
    }    
    createGround();
    
    FOREACH(list<PolyLine>, polylines, p){
      if(p->depth>0){
	createPolyline(*p);
      }
    }        
    obstacle_exists=true;
  };
  
  void ComplexPlayground::createPolyline(const PolyLine& polyline){
    typedef list<pair<Pos,Pos> > pospairs;
    pospairs pairs;
    Pos ps[2];
    int i=0;
    FOREACHC(list<Pos>, polyline.points, p){
      ps[i%2]=*p;
      if(i>=1){
	pairs.push_back(pair<Pos, Pos>(ps[(i-1)%2]*factor, ps[i%2]*factor));
      }
      i++;
    }
    FOREACHC(pospairs, pairs, p){
      Pos size = p->second - p->first;
      double length = sqrt(size.x()*size.x()+size.y()*size.y());
      Pos offset = (p->second + p->first)/2;
      Box* box = new Box( length, polyline.thickness*.03175*factor , polyline.depth*heightfactor);
      // Todo: use getTexture...
      box->setTexture(TextureDescr(wallTextureFileName,-1,-1));
      box->init(odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw);
      double angle = atan2(size.y(),size.x());
      box->setPose(osg::Matrix::rotate(angle,Pos(0,0,1)) *  osg::Matrix::translate(offset) * pose);
      obst.push_back(box);
    }
  }
}
