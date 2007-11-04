/*
** channelselectrow.h
** 
** Made by Georg Martius
** 
** Started on  Sun Oct  7 12:45:48 2007 Georg Martius
** Last update Sun Oct  7 12:45:48 2007 Georg Martius
*/

#ifndef   	CHANNELSELECTROW_H_
# define   	CHANNELSELECTROW_H_


#include <q3frame.h>
#include <q3boxlayout.h>
#include <qlabel.h>
#include <qlayout.h>

#include "taggedcombobox.h"

/**
  * \brief Container for widgets associated with one gnuplot channel.
  * \author Georg Martius
*/
class ChannelSelectRow : public Q3Frame
{
    Q_OBJECT
            
public:
    ChannelSelectRow(const Tag& tag, int buttons, QWidget* parent = 0, const char *name = 0);
    ~ChannelSelectRow();
    int getSelectedIndex(int);
    QString getSelected(int);
    void setSelected(int, int index);
    void setSelected(int, const QString s);
    QString getChannelName();

    void addItem(const QString&);

signals:
    void sendtaggedComboBoxChanged(const Tag&, int, const QString&);

private slots:
    void receiveComboBox(const Tag&, int, const QString&);

private:
    void init( QWidget *parent, int buttons);

private:
    QLabel* channelLabel;
    TaggedComboBox** ComboBoxList;
    int buttons;
    
    Tag channelName;

    Q3BoxLayout* layout;
};

#endif 	    /* !CHANNELSELECTROW_H_ */
