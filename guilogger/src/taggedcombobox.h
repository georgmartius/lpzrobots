/*
** taggedcombobox.h
** 
** Made by Georg Martius
** Login   <georg@agattu>
** 
** Started on  Sun Oct  7 12:36:17 2007 Georg Martius
** Last update Sun Oct  7 12:36:17 2007 Georg Martius
*/

#ifndef   	TAGGEDCOMBOBOX_H_
# define   	TAGGEDCOMBOBOX_H_


#include <qcombobox.h>

typedef QString Tag;

/**
  * \brief Combo which carries a tag. Useful for identification. Overwrites combobox  signals in order to include the tag.
  * \author Georg Martius
  */
class TaggedComboBox : public QComboBox
{
    Q_OBJECT

public:
    TaggedComboBox(QWidget* parent, const char* name = 0);
    TaggedComboBox(const Tag& tag, int gpwindow, QWidget* parent, const char* name = 0 );

    ~TaggedComboBox();

    void setTag(const Tag& tag);
    void setGPWindow(int i);
    int  getGPWindow();
    
private:
    void init();

private slots:
    void parenthighlighted ( const QString& entry );

signals:
    void taggedHighlighted(const Tag& tag, int gpwindow, const QString& entry);

private:
    Tag tag;
    int gpwindow;
};


#endif 	    /* !TAGGEDCOMBOBOX_H_ */
