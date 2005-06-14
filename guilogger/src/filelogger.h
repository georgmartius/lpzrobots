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

#include <qdatetime.h>
#include <qobject.h>
#include <qstring.h>

/** \brief Short class for logging char* strings to file named with date and time.
  * \author Dominic Schneider
  */
class FileLogger : public QObject
{
    Q_OBJECT

public:
    FileLogger(QString pf="");
    void setPrefix(QString pf) {prefix = pf;};

private:
    QString prefix;
    QString filename;

private slots:
    void writeChannelNames(char *);  // removes #C from the front of the input string      deprecated
    void writeChannelData(char *);   // writes the block as it gets it to file

};
