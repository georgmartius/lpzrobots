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
 *   Revision 1.8  2011-04-06 06:49:09  guettler
 *   - main window activates when hovering with mouse, enabling keyboard shortcuts without click onto the window
 *
 *   Revision 1.7  2011/04/05 12:16:04  guettler
 *   - new tabWidget
 *   - all found DNS devices are shown in tabWidget with a QDNSDeviceWidget each
 *   - QDNSDeviceWidget shows DNS device name, USB adapter name and type,
 *     response time and incoming/outgoing status (if messages are currently sent
 *     or received)
 *
 *   Revision 1.6  2011/01/24 16:58:25  guettler
 *   - QMessageDispatchServer is now informed when client app closes itself
 *   - QMessageDispatchWindow actually closes if client app closes itself
 *   - hint: this should late be
 *
 *   Revision 1.5  2010/11/23 11:08:06  guettler
 *   - some helper functions
 *   - bugfixes
 *   - better event handling
 *
 *   Revision 1.4  2010/11/19 15:15:00  guettler
 *   - new QLog feature
 *   - bugfixes
 *   - FT232Manager is now in lpzrobots namespace
 *   - some cleanups
 *
 *   Revision 1.3  2010/11/18 16:58:18  wrabe
 *   - current state of work
 *
 *   Revision 1.2  2010/11/14 20:39:37  wrabe
 *   - save current developent state
 *
 *   Revision 1.1  2010/11/11 15:35:59  wrabe
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
#include "QExtAction.h"
#include "QLog.h"
#include "constants.h"
#include "QCCHelper.h"

namespace lpzrobots {

  class QMessageDispatchWindow : public QMainWindow {

    Q_OBJECT

    public:
      QMessageDispatchWindow(QString applicationPath);
      virtual ~QMessageDispatchWindow();

      virtual QAbstractMessageDispatchServer* getQMessageDispatchServer() {
        return (QAbstractMessageDispatchServer*) &this->messageDispatcher;
      }

      enum LOG_TYPE {
        LOG_ERROR, LOG_WARNING, LOG_VERBOSE, LOG_DEBUG,
      };

    protected:
      void closeEvent(QCloseEvent *event);
      void enterEvent(QEvent* event);

    signals:
      void sig_quitServer();

    private slots:
      void sl_TextLog(QString s);
      void sl_eventHandler(int eventCode);
      void sl_statusLabelTimerExpired();
      void sl_Close();
      void sl_updateDNSDeviceWidgets();

    private:

      void createActions();
      void createMenus(int applicationMode = APPLICATION_MODE_None);
      void readSettings();
      void writeSettings();
      void sleep(ulong msecs);
      int addDNSDeviceWidgetsToGrid(QList<QCCHelper::DNSDevice_t*> deviceList, QGridLayout* grid);

      // TODO:
      // the future is to combine the two methods ...
      // first is to rewrite the protocoll of the isp-adapter
      void DispatchMessage(QByteArray msg);

      void setMode(int mode);
      int getDefaultBaudrateByName(QString actDeviceName);

      QTabWidget *tabWidget;
      QLogViewWidget *logView;
      QLabel* statusLabel;
      QTimer statusLabelTimer;
      QECBMessageDispatchServer messageDispatcher;
      QLog* qlog;
      QWidget* dnsDeviceContainerWidget;

      QExtAction *action_Exit;
      QExtAction *action_About;
      QExtAction *action_ScanUsbDevices;
      QExtAction *action_ClearLogView;
      QExtAction *action_PrintDNSTable;
      QExtAction *action_NDDiscoverTimeout;

      QMenu *menu_File;
      QMenu *settingsMenu;
      QMenu *menu_Help;

      QString applicationPath;

      enum ACTION_EVENT {
        //---------------------------------------
        EVENT_APPLICATION_LOGVIEW_CLEAR,
        EVENT_APPLICATION_CLOSE,
        EVENT_APPLICATION_ABOUT,
        EVENT_APPLICATION_SCAN_USBDEVICE,
        EVENT_APPLICATION_PRINT_DNS_TABLE,
        EVENT_APPLICATION_SET_NODEDISCOVER_TIMEOUT
      //---------------------------------------
      };

  };

} // namespace lpzrobots
#endif // __QMESSAGEDISPATCHWINDOW_H
