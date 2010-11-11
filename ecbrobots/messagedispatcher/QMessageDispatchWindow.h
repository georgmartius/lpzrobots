/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    jhoffmann@informatik.uni-leipzig.de                                  *
 *    wolfgang.rabe@01019freenet.de                                        *
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
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2010-11-11 15:35:59  wrabe
 *   -current development state of QMessageDispatchServer
 *   -introduction of QCommunicationChannels and QCCHelper
 *                                                *
 *                                                                         *
 ***************************************************************************/

#ifndef __QMESSAGEDISPATCHWINDOW_H
#define __QMESSAGEDISPATCHWINDOW_H

#include <QtGui>
#include <QMainWindow>
#include <QTextEdit>
#include <QTimer>
#include <QStyleOptionProgressBarV2>
#include <qdom.h>
#include "types.h"
#include "QLogViewWidget.h"
#include "QECBMessageDispatchServer.h"
#include "QAbstractMessageClient.h"
#include "constants.h"

namespace lpzrobots {

class QMessageDispatchWindow : public QMainWindow {

  Q_OBJECT

  public:
  QMessageDispatchWindow(QString applicationPath);
    QString applicationPath;

    virtual QAbstractMessageDispatchServer* getQMessageDispatchServer() {
      return &this->messageDispatcher;
    }
  protected:
    void closeEvent(QCloseEvent *event);

    signals:
    void sig_quitServer();


  private slots:
    void sl_TextLog(QString s);
    void sl_DispatchMessage(QByteArray msg);
    void sl_Close();
    void sl_ClearLogView();
    void sl_About();
    void sl_TimerExpired();

  private:

    void createActions();
    void createMenus(int applicationMode=APPLICATION_MODE_None);
    void readSettings();
    void writeSettings();
    void sleep(ulong msecs);

    // TODO:
    // the future is to combine the two methods ...
    // first is to rewrite the protocoll of the isp-adapter
    void DispatchMessage(QByteArray msg);

    void setMode(int mode);
    int getDefaultBaudrateByName(QString actDeviceName);

    QTabWidget *tabWidget;
    QLogViewWidget *logView;
    QTimer *timer;


    QMenu *fileMenu;
    QMenu *helpMenu;


    // --- Actions ----------------------------------
    // File-Menu
    QAction *action_Exit;
    // Help-Menu
    QAction *action_About;



};

} // namespace lpzrobots
#endif // __QMESSAGEDISPATCHWINDOW_H
