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
#ifndef TAGGEDCHECKBOX_H
#define TAGGEDCHECKBOX_H

#include <qcheckbox.h>

typedef QString Tag;

/**
  * \brief Checkbox which carries a tag. Useful for identification. Overwrites checkbox  signals in order to include the tag.
  * \author Dominic Schneider
  */
class TaggedCheckBox : public QCheckBox
{
    Q_OBJECT

public:
    TaggedCheckBox(QWidget* parent, const char* name = 0);
    TaggedCheckBox(const Tag& tag, int gpwindow, QWidget* parent, const char* name = 0 );

    ~TaggedCheckBox();

    void setTag(const Tag& tag);
    void setGPWindow(int i);
    int  getGPWindow();
    
private:
    void init();

private slots:
    void parenttoggled ( bool on );

signals:
    void taggedToggled(const Tag& tag, int gpwindow, bool on);

private:
    Tag tag;
    int gpwindow;
};

#endif
