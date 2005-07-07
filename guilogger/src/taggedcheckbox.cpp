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
#include "taggedcheckbox.h"

TaggedCheckBox::TaggedCheckBox(QWidget* parent, const char* name)
 : QCheckBox(parent, name)
{
    init();
}

TaggedCheckBox::TaggedCheckBox(const Tag& tag, int gpwindow, QWidget* parent, const char* name)
    : QCheckBox(parent, name)
{
    this->tag=tag;
    this->gpwindow =  gpwindow;
    setOn(FALSE);
    init();
}

void TaggedCheckBox::init(){
    connect(this, SIGNAL(toggled(bool)), this, SLOT(parenttoggled(bool)));
}

TaggedCheckBox::~TaggedCheckBox()
{
}

void TaggedCheckBox::setTag(const Tag& tag){
    this->tag=tag;
}

void TaggedCheckBox::parenttoggled ( bool on ){
    emit taggedToggled(tag, gpwindow, on);
}

int TaggedCheckBox::getGPWindow()
{   return gpwindow;
}
