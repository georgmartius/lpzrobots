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
 *   Revision 1.2  2011-01-04 12:00:46  guettler
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
      QGridPos() : QPoint() {}
      QGridPos(int row, int column) : QPoint(column,row) {}
      QGridPos(const QGridPos& gridPos) : QPoint(gridPos.x(), gridPos.y()) {}
      virtual ~QGridPos() {}

      virtual inline int row() const { return y(); }
      virtual inline int column() const { return x(); }

      friend inline bool operator<(const QGridPos& p1, const QGridPos& p2) {
        return ((p1.row() < p2.row()) || (p1.row() == p2.row() && p1.column() < p2.column()));
      }

    protected:
      using QPoint::x;
      using QPoint::y;
  };

}

#endif /* __QGRIDPOS_H_ */
