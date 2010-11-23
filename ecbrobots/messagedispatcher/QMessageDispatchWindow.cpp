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
 *   Revision 1.5  2010-11-23 11:08:06  guettler
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
 *                                              *
 *                                                                         *
 ***************************************************************************/

#include "QMessageDispatchWindow.h"

namespace lpzrobots {

  QMessageDispatchWindow::QMessageDispatchWindow(QString applicationPath) {
    this->applicationPath = applicationPath;
    this->setWindowTitle("ECBMessageDispatchWindow");
    logView = new QLogViewWidget();
    statusLabel = new QLabel(statusBar());
    statusBar()->addWidget(statusLabel);
    connect(&statusLabelTimer, SIGNAL(timeout()), this, SLOT(sl_statusLabelTimerExpired()));

    // logging function
    qlog = new QLog(applicationPath);
    connect(qlog, SIGNAL(sig_textLog(QString)), this, SLOT(sl_TextLog(QString)));

    QLog::logVerbose("ApplicationPath='" + applicationPath + "'");

    setPalette(QPalette(QColor(200, 230, 200)));

    // Layout:
    //------------------------------------------------------
    QGridLayout *grid = new QGridLayout();
    QWidget *mainpanel = new QWidget();
    mainpanel->setLayout(grid);
    setCentralWidget(mainpanel);

    tabWidget = new QTabWidget;
    tabWidget->addTab(logView, tr("Report"));
    grid->addWidget(tabWidget, 0, 0);

    setMinimumWidth(900);
    setMaximumWidth(900);



    createActions();
    createMenus();
    readSettings();

    connect(this, SIGNAL(sig_quitServer()), &messageDispatcher, SIGNAL(sig_quitServer()));

  }

  QMessageDispatchWindow::~QMessageDispatchWindow() {
    //messageDispatcher->~QECBMessageDispatchServer();
    tabWidget->~QObject();
    logView->~QLogViewWidget();
    action_ScanUsbDevices->~QExtAction();
    action_About->~QExtAction();
    action_Exit->~QExtAction();
    menu_File->~QMenu();
    menu_Help->~QMenu();
    delete qlog;
  }

  void QMessageDispatchWindow::createActions() {
    action_ScanUsbDevices = new QExtAction(EVENT_APPLICATION_SCAN_USBDEVICE, tr("ScanUSBDevices"), this);
    action_ScanUsbDevices->setStatusTip(tr("Scan for connected USB-Tools."));
    action_ScanUsbDevices->setShortcut(Qt::Key_F5);
    action_ScanUsbDevices->setEnabled(true);
    connect(action_ScanUsbDevices, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler(int)));

    action_ClearLogView = new QExtAction(EVENT_APPLICATION_LOGVIEW_CLEAR, tr("Clear"), this);
    action_ClearLogView->setStatusTip(tr("Remove all the text-lines from LogView."));
    action_ClearLogView->setShortcut(Qt::Key_C | Qt::CTRL | Qt::SHIFT);
    action_ClearLogView->setEnabled(true);
    connect(action_ClearLogView, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler(int)));

    action_PrintDNSTable = new QExtAction(EVENT_APPLICATION_PRINT_DNS_TABLE, tr("Printout DNS-Table"), this);
    action_PrintDNSTable->setStatusTip(tr("Prints out the DNS-Table ([dnsName]->[usbDeviceName])."));
    action_PrintDNSTable->setShortcut(Qt::Key_P | Qt::CTRL);
    action_PrintDNSTable->setEnabled(true);
    connect(action_PrintDNSTable, SIGNAL(triggered()), &messageDispatcher, SLOT(sl_printDNSDeviceToQCCMap()));

    action_Exit = new QExtAction(EVENT_APPLICATION_CLOSE, tr("&Quit"), this);
    action_Exit->setShortcut(tr("Ctrl+Q"));
    action_Exit->setStatusTip(tr("Exit the application"));
    connect(action_Exit, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler(int)));

    action_NDDiscoverTimeout = new QExtAction(EVENT_APPLICATION_SET_NODEDISCOVER_TIMEOUT, tr("&NodeDiscover timeout"), this);
    action_NDDiscoverTimeout->setStatusTip(tr("Set the XBee's Node Discover timeout"));
    connect(action_NDDiscoverTimeout, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler(int)));

    // Actions About
    action_About = new QExtAction(EVENT_APPLICATION_ABOUT, tr("&About"), this);
    action_About->setStatusTip(tr("Show the application's About box"));
    connect(action_About, SIGNAL(triggered(int)), this, SLOT(sl_eventHandler(int)));

  }

  void QMessageDispatchWindow::createMenus(int applicationMode) {
    //delete this->menuBar();
    this->menuBar()->clear();

    switch (applicationMode) {
      default: {
        menu_File = menuBar()->addMenu(tr("&File"));
        menu_File->addAction(action_ScanUsbDevices);
        menu_File->addAction(action_PrintDNSTable);
        menu_File->addAction(action_ClearLogView);
        menu_File->addSeparator();
        menu_File->addAction(action_Exit);

        settingsMenu = menuBar()->addMenu(tr("Settings"));
        qlog->addActionsToMenu(settingsMenu);

        menu_Help = menuBar()->addMenu(tr("&Help"));
        menu_Help->addAction(action_About);
        break;
      }
    }
  }

  void QMessageDispatchWindow::readSettings() {
    QSettings settings(applicationPath + QString("messagedispatcher.settings"), QSettings::IniFormat);
    QPoint pos = settings.value("pos", QPoint(200, 200)).toPoint();
    QSize size = settings.value("size", QSize(400, 400)).toSize();
    resize(size);
    move(pos);
  }

  void QMessageDispatchWindow::writeSettings() {
    QSettings settings(applicationPath + QString("messagedispatcher.settings"), QSettings::IniFormat);
    settings.setValue("pos", pos());
    settings.setValue("size", size());
  }

  void QMessageDispatchWindow::sl_eventHandler(int eventCode) {

    switch (eventCode) {
      case EVENT_APPLICATION_LOGVIEW_CLEAR: {
        logView->clearLogViewText();
        break;
      }
      case EVENT_APPLICATION_CLOSE: {
        close();
        break;
      }
      case EVENT_APPLICATION_ABOUT: {
        QMessageBox::about(
            this,
            tr("About the Application"),
            tr(
                "ECB_Robot-Application V2.0, Tool to connect real robots (containing an ecb) onto a neuro-controller located on a standard pc."));
        break;
      }
      case EVENT_APPLICATION_SCAN_USBDEVICE: {
        messageDispatcher.scanUsbDevices();
        break;
      }
      default:
        break;
    }
  }

  void QMessageDispatchWindow::closeEvent(QCloseEvent *event) {
    writeSettings();
    emit
    sig_quitServer();
    event->accept();
  }

  void QMessageDispatchWindow::sleep(ulong msecs) {
    QTime dieTime = QTime::currentTime().addMSecs(msecs);
    while (QTime::currentTime() < dieTime)
      QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
  }

  void QMessageDispatchWindow::sl_TextLog(QString sText) {
    statusLabelTimer.stop();
    statusLabel->setText(sText);
    statusLabelTimer.start(5000);
    logView->appendLogViewText(sText);
  }

  void QMessageDispatchWindow::sl_statusLabelTimerExpired() {
    statusLabel->setText("");
  }

} // namespace lpzrobots
