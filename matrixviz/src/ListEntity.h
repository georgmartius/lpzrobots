/*
 * ListEntity.h
 *
 *  Created on: 13.01.2010
 *      Author: oni
 */

#include <QColor>
#include <QtGui>

#ifndef __LISTENTITY_H_
#define __LISTENTITY_H_

class ListEntity : public QWidget{

  Q_OBJECT

  public:
    ListEntity(int i, QColor color, double pos, QWidget *parent = 0);
    ~ListEntity();

    int i;
    QColor color;
    double pos;
    QPushButton* button;
    QLineEdit* lineEdit;

  public slots:
    void addClicked();
    void remClicked();
    void changeColor();
    void changePos(const QString &text);

  signals:
    void addClicked(int i);
    void remClicked(int i);
    void changeColor(int i, QColor color);
    void changePos(int i, double pos);

  private:
    void fillButton();
};

#endif /* __LISTENTITY_H_ */
