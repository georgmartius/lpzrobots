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

//#include "guilogger.h"

ChannelRow::ChannelRow(const Tag& channelName, int buttons, QWidget *parent, const char *name)
    : QFrame(parent, name)
{
    this->channelName = channelName;
    init(parent, buttons);
}


ChannelRow::~ChannelRow()
{   delete []CheckBoxList;
}


void ChannelRow::init( QWidget *parent, int buttons){
    this->buttons = buttons;
    layout = new QBoxLayout(this, QBoxLayout::LeftToRight);
    this->channelLabel = new QLabel(channelName, this);
    layout->addWidget(channelLabel);
    CheckBoxList = new TaggedCheckBox*[buttons];
    
    for(int i=0; i<buttons; i++)
    {  CheckBox = new TaggedCheckBox( channelName, i, this, channelName);
       CheckBoxList[i] = CheckBox;
       layout->addWidget(CheckBox);
//       connect(CheckBox,SIGNAL(taggedToggled(const Tag&, int, bool)),
//            parent, SLOT(taggedCheckBoxToggled(const Tag&, int, bool)));
       connect(CheckBox,SIGNAL(taggedToggled(const Tag&, int, bool)),      // Signal from one of the Checkboxes
               this, SLOT(receiveCheckedBox(const Tag&, int, bool)));

    }

}

// SLOT : wird nur durchgeschliffen
void ChannelRow::receiveCheckedBox(const Tag& Tag, int i, bool b)
{    emit sendtaggedCheckBoxToggled(Tag, i, b);
}

QString ChannelRow::getChannelName()
{  return channelLabel->text();
}


bool ChannelRow::isChecked(int checkbox)
{   
    if(checkbox > buttons || checkbox < 0) return FALSE;
    return CheckBoxList[checkbox]->isChecked();
}


void ChannelRow::setChecked(int gpwindow, bool check)
{
    if(gpwindow > buttons || gpwindow < 0) return;
    CheckBoxList[gpwindow]->setChecked(check);
}
