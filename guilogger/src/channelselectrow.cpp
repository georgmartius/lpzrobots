/***************************************************************************
 *   Copyright (C) 2007 by Georg Martius   *
 *      *
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
#include "channelselectrow.h"
//Added by qt3to4:
#include <Q3Frame>
#include <QLabel>


ChannelSelectRow::ChannelSelectRow(const Tag& channelName, int buttons, QWidget *parent, const char *name)
    : Q3Frame(parent, name)
{
    this->channelName = channelName;
    init(parent, buttons);
}


ChannelSelectRow::~ChannelSelectRow()
{   delete []ComboBoxList;
}


void ChannelSelectRow::init( QWidget*, int buttons){
  this->buttons = buttons;
  layout = new Q3BoxLayout(this, Q3BoxLayout::LeftToRight);
  this->channelLabel = new QLabel(channelName, this);
  layout->addWidget(channelLabel);
  ComboBoxList = new TaggedComboBox*[buttons];    
  
  TaggedComboBox* ComboBox;
  for(int i=0; i<buttons; i++)
    {  
      ComboBox = new TaggedComboBox( channelName, i, this, channelName);
      ComboBox->setMinimumContentsLength(3);
      ComboBox->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLength);
      ComboBoxList[i] = ComboBox;
      layout->addWidget(ComboBox);
      connect(ComboBox,SIGNAL(taggedHighlighted(const Tag&, int, const QString&)),
	      // Signal from one of the Comboboxes
	      this, SLOT(receiveComboBox(const Tag&, int, const QString&)));
      
    }
  
}

// SLOT : wird nur durchgeschliffen
void ChannelSelectRow::receiveComboBox(const Tag& Tag, int i, const QString& e)
{    emit sendtaggedComboBoxChanged(Tag, i, e);
}

QString ChannelSelectRow::getChannelName()
{  return channelLabel->text();
}


QString ChannelSelectRow::getSelected(int combobox)
{   
    if(combobox > buttons || combobox < 0) return FALSE;
    return ComboBoxList[combobox]->currentText();
}


void ChannelSelectRow::setSelected(int gpwindow, int index)
{
    if(gpwindow > buttons || gpwindow < 0) return;
    ComboBoxList[gpwindow]->setCurrentIndex(index);
}

void ChannelSelectRow::addItem(const QString& e){
  for(int i=0; i<buttons; i++)
    ComboBoxList[i]->addItem(e);
    
}

