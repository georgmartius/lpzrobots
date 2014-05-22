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
#ifndef __COMPLEXPLAYGROUND_H
#define __COMPLEXPLAYGROUND_H

#include "mathutils.h"
#include "abstractground.h"
#include "pos.h"

namespace lpzrobots {

// Taken from XFIG documentation
// .....
// (2) The first non-comment line consists of the following:

//         string  orientation             ("Landscape" or "Portrait")
//         string  justification           ("Center" or "Flush Left")
//         string  units                   ("Metric" or "Inches")
//         string  papersize               ("Letter", "Legal", "Ledger", "Tabloid",
//                                          "A", "B", "C", "D", "E",
//                                          "A4",   "A3", "A2", "A1", "A0" and "B5")
//         float   magnification           (export and print magnification, %)
//         string  multiple-page           ("Single" or "Multiple" pages)
//         int     transparent color       (color number for transparent color for GIF
//                                          export. -3=background, -2=None, -1=Default,
//                                          0-31 for standard colors or 32- for user colors)
//         # optional comment              (An optional set of comments may be here,
//                                          which are associated with the whole figure)
//         int     resolution coord_system (Fig units/inch and coordinate system:
//                                            1: origin at lower left corner (NOT USED)
//                                            2: upper left)

//     Fig_resolution is the resolution of the figure in the file.
//     Xfig will always write the file with a resolution of 1200ppi so it
//     will scale the figure upon reading it in if its resolution is different
//     from 1200ppi.  Pixels are assumed to be square.

//     Xfig will read the orientation string and change the canvas to match
//     either the Landscape or Portrait mode of the figure file.

//     The units specification is self-explanatory.

//     The coordinate_system variable is ignored - the origin is ALWAYS the
//     upper-left corner.

//     ** Coordinates are given in "fig_resolution" units.
//     ** Line thicknesses are given in 1/80 inch (0.3175mm) or 1 screen pixel.
//        When exporting to EPS, PostScript or any bitmap format (e.g. GIF),  the
//        line thickness is reduced to 1/160 inch (0.159mm) to "lighten" the look.
//     ** dash-lengths/dot-gaps are given in 80-ths of an inch.




// (3) The rest of the file contains various objects.  An object can be one
//     of six classes (or types).

//         0)      Color pseudo-object.
//         1)      Ellipse which is a generalization of circle.
//         2)      Polyline which includes polygon and box.
//         3)      Spline which includes
//                 closed/open approximated/interpolated/x-spline spline.
//         4)      Text.
//         5)      Arc.
//         6)      Compound object which is composed of one or more objects.
// ........
//    (3.5) POLYLINE

//     First line:
//         type    name                    (brief description)
//         ----    ----                    -------------------
//         int     object_code             (always 2)
//         int     sub_type                (1: polyline
//                                          2: box
//                                          3: polygon
//                                          4: arc-box)
//                                          5: imported-picture bounding-box)
//         int     line_style              (enumeration type)
//         int     thickness               (1/80 inch)
//         int     pen_color               (enumeration type, pen color)
//         int     fill_color              (enumeration type, fill color)
//         int     depth                   (enumeration type)
//         int     pen_style               (pen style, not used)
//         int     area_fill               (enumeration type, -1 = no fill)
//         float   style_val               (1/80 inch)
//         int     join_style              (enumeration type)
//         int     cap_style               (enumeration type, only used for POLYLINE)
//         int     radius                  (1/80 inch, radius of arc-boxes)
//         int     forward_arrow           (0: off, 1: on)
//         int     backward_arrow          (0: off, 1: on)
//         int     npoints                 (number of points in line)

//     Forward arrow line: same as ARC object

//     Backward arrow line: same as ARC object

//     For picture (type 5) the following line follows:
//         type    name                    (brief description)
//         ----    ----                    -------------------
//         boolean flipped                 orientation = normal (0) or flipped (1)
//         char    file[]                  name of picture file to import

//     Points line(s).  The x,y coordinates follow, any number to a line, with
//     as many lines as are necessary:
//         type    name                    (brief description)
//         ----    ----                    -------------------
//         int     x1, y1                  (Fig units)
//         int     x2, y2                  (Fig units)
//           .
//           .
//         int     xnpoints ynpoints       (this will be the same as the 1st
//                                         point for polygon and box)

  class PolyLine {
  public:
    PolyLine(){
      object_code = 0;
      sub_type    = 0;
      line_style  = 0;
      thickness   = 0;
      depth       = 0;       //             (multiple of height)
    };
    /** try to parses a PolyLine at the start of the list of lines and returns the consumed lines
        1: no success; >=2 success
     */
    int parse(std::list<char*> lines);
    void print();

    int     object_code; //             (always 2)
    int     sub_type;    //             (we require 1: polyline)
    int     line_style;  //             (0: normal wall, 1: border, rest ignored)
    int     thickness;   //             (1/80 inch means factor of .3175 )
    int     depth;       //             (multiple of height)
    std::list<Pos> points;
  };

  /** Playground that uses an xfig file with polylines
      linetype 0 is normal wall
      linetype 1 is border
      thickness is used as well, thickness is multiplied with wallthickness.
      depth is used as a height value and is multiplied with heightfactor.
      The entire size can be scaled with a global factor
  */
  class ComplexPlayground : public AbstractGround {

  protected:

    std::string filename;
    double factor, heightfactor;

    std::list<PolyLine> polylines;

  public:
    /**
       @param factor global factor for the entire playground
       @heightfactor factor for depth value of xfig line to determine wall height
     */
    ComplexPlayground(const OdeHandle& odeHandle, const OsgHandle& osgHandle ,
                      const std::string& filename,
                      double factor = 1, double heightfactor=0.02, bool createGround=true);

    void createPolyline(const PolyLine&);

  protected:
    virtual void create();

  };

}

#endif
