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
#include "channelrow.h"

/*ChannelRow::ChannelRow(guilogger *parent, const char *name)
    : QFrame(parent, name)
{

}*/

ChannelRow::ChannelRow(const Tag& channelName, int buttons, guilogger *parent, const char *name)
    : QFrame(parent, name)
{
    this->channelName = channelName;
    init(parent, buttons);
}

ChannelRow::~ChannelRow()
{
}

void ChannelRow::init( guilogger *parent, int buttons){
    layout = new QBoxLayout(this, QBoxLayout::LeftToRight);
    this->channelLabel = new QLabel(channelName, this);
    layout->addWidget(channelLabel);

    for(int i=0; i<buttons; i++)
    {  CheckBox = new TaggedCheckBox( channelName, i, this, channelName);
       layout->addWidget(CheckBox);
       connect(CheckBox,SIGNAL(taggedToggled(const Tag&, int, bool)),
            parent, SLOT(taggedCheckBoxToggled(const Tag&, int, bool)));
    }

}

