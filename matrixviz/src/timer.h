/*
 * timer.h
 *
 * --just for testing--
 *
 *  Created on: 15.10.2009
 *      Author: oni
 */

#ifndef __TIMER_H_
#define __TIMER_H_

#include <QThread>
#include <QTimer>
#include "VisualiserSubWidget.h"

class Timer : public QThread{

  Q_OBJECT

public:
  Timer(VisualiserSubWidget *vis, QObject *parent = 0) : QThread(parent){
    this->vis = vis;
  }
     ~Timer(){ delete timer;}

     void run(void)
     {
        timer = new QTimer;
        timer->setSingleShot(true);
        timer->setInterval(1000);

        connect( timer, SIGNAL(timeout()), this, SLOT(restartTimer()),Qt::DirectConnection);
        connect( timer, SIGNAL(timeout()), vis, SLOT(updateViewableChannels()));

        timer->start();

        exec();

     }

  private slots:
     void restartTimer(void)
     {
        timer->start(); //funkst!!
     }

  private:
     QTimer *timer;
     VisualiserSubWidget *vis;
  };

#endif /* __TIMER_H_ */
