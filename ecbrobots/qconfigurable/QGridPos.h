/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    jhoffmann@informatik.uni-leipzig.de                                  *
 *    wolfgang.rabe@01019freenet.de                                        *
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
 *   Revision 1.3  2011-01-05 13:29:21  guettler
 *   - code cleaned up, including performance enhancements
 *
 *   Revision 1.2  2011/01/04 12:00:46  guettler
 *   -bughunting
 *
 *   Revision 1.1  2010/12/16 16:39:25  wrabe
 *   - drag&drop reworked: user can now drag a parameter to a any place
 *   - rearrangement of parameters now made only when user wants this
 *   - bugfixes
 *
 *                                                                         *
 ***************************************************************************/

#ifndef __QGRIDPOS_H_
#define __QGRIDPOS_H_

#include <qpoint.h>

namespace lpzrobots {
  
  class QGridPos : public QPoint {
    public:
      inline QGridPos() : QPoint() {}
      inline QGridPos(int row, int column) : QPoint(column,row) {}

      inline int row() const { return y(); }
      inline int column() const { return x(); }

      inline bool operator<(const QGridPos& p2) const {
        return ((row() < p2.row()) || (row() == p2.row() && column() < p2.column()));
      }

    protected:
      using QPoint::x;
      using QPoint::y;
  };

}

#endif /* __QGRIDPOS_H_ */
