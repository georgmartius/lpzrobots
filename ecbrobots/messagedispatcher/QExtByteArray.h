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
 *   Revision 1.1  2010-11-14 20:39:37  wrabe
 *   - save current developent state
 *
 *                                                                         *
 ***************************************************************************/

#ifndef __QEXTBYTEARRAY_H_
#define __QEXTBYTEARRAY_H_

#include <QByteArray>

namespace lpzrobots {

  class QExtByteArray : public QByteArray {

    public:
      QExtByteArray();
      virtual ~QExtByteArray();

      virtual void append(uchar c);
      virtual void appendEscaped(uchar c);
      virtual void appendEscapedChecksum(uchar c);
      virtual void appendChecksum();

      virtual void clear();

    private:
      uchar checksum;

  };

} // namespace lpzrobots

#endif /* __QEXTBYTEARRAY_H_ */
