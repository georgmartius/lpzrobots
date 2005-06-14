/***************************************************************************
 *   Copyright (C) 2005 by Dominic Schneider   *
 *   dominic@isyspc8   *
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
 ***************************************************************************/
#include "qdatasource.h"
#include <qthread.h>
#include <qstring.h>

/** \brief Class for reading complete lines from the serial port terminated by a special character
  * \author Dominic Schneider and someone unknowen who wrote the algorithm
  */
class QSerialReader : public QDataSource
{
    Q_OBJECT

private:
    QString port;
    int baudrate;
    char blockterminator;  // end-of-line symbol for readed data

public:
    QSerialReader(char bt = '\n');
    virtual void run();

    void setComport(char *port){ this->port = port; };   /// set com port
    void setBaudrate(int baud){ baudrate = baud; };      /// set baud rate

};
