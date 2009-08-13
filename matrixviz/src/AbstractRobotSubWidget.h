/***************************************************************************
 *   Copyright (C) 2008 by mc   *
 *   mc@linux-6hav   *
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
#ifndef ABSTRACT_ROBOT_SUB_WIDGET_H
#define ABSTRACT_ROBOT_SUB_WIDGET_H

#include <QtGui>

class AbstractPlotChannel;

#include "AbstractPlotChannel.h"

#include <list>
#include <iostream>
#include <string>

class AbstractRobotSubWidget: public QWidget
{

	Q_OBJECT

public:
	AbstractRobotSubWidget(QWidget *parent = 0) : QWidget(parent)
	{
		//      channelList.clear();
	};

	void addPlotChannel(AbstractPlotChannel* c)
	{ // default variant
		this->channelList.push_back(c);
	}

public slots:

/**
 * update widgets graphical displayed stuff
 */
virtual void updateViewableChannels() = 0;

protected:
	std::list<AbstractPlotChannel*> channelList;

private:

};

#endif
