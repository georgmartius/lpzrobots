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
#include <qobject.h>
#include <qstring.h>
#include <qstringlist.h>

/** \brief  Class for parsing the data you want out of a string
  * \author Dominic Schneider
  */
class QParseData : public QObject
{
    Q_OBJECT

private:
//    std::list<QString> nameslist;

public:
    QParseData();

signals:
    void sendGNUPlotData(    QStringList &data);
    void sendGNUPlotChannels( QStringList &names);

public slots:
    void parseData(char *);

};
