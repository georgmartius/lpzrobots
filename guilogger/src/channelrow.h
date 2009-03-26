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
#ifndef CHANNELROW_H
#define CHANNELROW_H

#include <q3frame.h>
#include <q3boxlayout.h>
#include <qlabel.h>
#include <qlayout.h>

#include "taggedcheckbox.h"

/**
  * \brief Container for widgets associated with one gnuplot channel.
  * \author Dominic Schneider
*/
class ChannelRow : public Q3Frame
{
    Q_OBJECT
            
public:
    ChannelRow(const Tag& tag, int buttons, QWidget* parent = 0, const char *name = 0);
    ~ChannelRow();
    bool isChecked(int );
    void setChecked(int, bool);
    QString getChannelName();

    virtual QSize sizeHint() const;  

signals:
    void sendtaggedCheckBoxToggled(const Tag&, int, bool);

private slots:
    void receiveCheckedBox(const Tag&, int, bool);

private:
    void init( QWidget *parent, int buttons);

private:
    QLabel* channelLabel;
    TaggedCheckBox** CheckBoxList;
    int buttons;
    
    Tag channelName;

    Q3BoxLayout* layout;
};

#endif
