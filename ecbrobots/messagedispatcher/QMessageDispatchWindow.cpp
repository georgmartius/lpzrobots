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
 *                                              *
 *                                                                         *
 ***************************************************************************/

#include "QMessageDispatchWindow.h"

namespace lpzrobots {

  QMessageDispatchWindow::QMessageDispatchWindow(QString applicationPath) {
    this->applicationPath = applicationPath;
    this->setWindowTitle("ECBMessageDispatchWindow");
    logView = new QLogViewWidget();
    timer = new QTimer();

    logView->appendLogViewText("ApplicationPath='" + applicationPath + "'");

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

    setMinimumWidth(600);
    setMaximumWidth(600);

    createActions();
    createMenus();
    readSettings();

    connect(timer, SIGNAL(timeout()), this, SLOT(sl_TimerExpired()));
    connect(this, SIGNAL(sig_quitServer()), &messageDispatcher, SIGNAL(sig_quitServer()));

  }
  void QMessageDispatchWindow::createActions() {
    action_Exit = new QAction(tr("&Quit"), this);
    action_Exit->setShortcut(tr("Ctrl+Q"));
    action_Exit->setStatusTip(tr("Exit the application"));
    connect(action_Exit, SIGNAL(triggered()), this, SLOT(close()));

    // Actions About
    action_About = new QAction(tr("&About"), this);
    action_About->setStatusTip(tr("Show the application's About box"));
    connect(action_About, SIGNAL(triggered()), this, SLOT(sl_About()));

  }
  void QMessageDispatchWindow::createMenus(int applicationMode) {
    //delete this->menuBar();
    this->menuBar()->clear();

    switch (applicationMode)
    {
      default:
      {
        fileMenu = menuBar()->addMenu(tr("&File"));
        fileMenu->addSeparator();
        fileMenu->addAction(action_Exit);

        helpMenu = menuBar()->addMenu(tr("&Help"));
        helpMenu->addAction(action_About);
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

  void QMessageDispatchWindow::closeEvent(QCloseEvent *event) {
    writeSettings();
    emit sig_quitServer();
    event->accept();
  }
  void QMessageDispatchWindow::sleep(ulong msecs) {
    QTime dieTime = QTime::currentTime().addMSecs(msecs);
    while (QTime::currentTime() < dieTime)
      QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
  }

  void QMessageDispatchWindow::sl_TextLog(QString sText) {
    statusBar()->showMessage(sText, 5000);
    logView->appendLogViewText(sText);
  }
  void QMessageDispatchWindow::sl_DispatchMessage(QByteArray receiveBuffer) {

    if (false)
    {
      printBuffer(receiveBuffer);
    }
  }

  void QMessageDispatchWindow::sl_ClearLogView() {
    logView->clearLogViewText();
  }
  void QMessageDispatchWindow::sl_Close() {
    close();
  }
  void QMessageDispatchWindow::sl_About() {
    QMessageBox::about(this, tr("About the Application"), tr(
        "ECB_Robot-Application V2.0, Tool to connect real robots (containing an ecb) onto a neuro-controller located on a standard pc."));
  }
  void QMessageDispatchWindow::sl_TimerExpired() {
    timer->stop();
    sl_TextLog("sl_TimerExpired");
  }

} // namespace lpzrobots
