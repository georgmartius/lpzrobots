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
 *   Revision 1.3  2011-01-24 16:26:36  guettler
 *   - new QLog feature ensures better performance if log levels are disabled
 *
 *   Revision 1.2  2010/11/23 11:08:06  guettler
 *   - some helper functions
 *   - bugfixes
 *   - better event handling
 *
 *   Revision 1.1  2010/11/19 15:15:00  guettler
 *   - new QLog feature
 *   - bugfixes
 *   - FT232Manager is now in lpzrobots namespace
 *   - some cleanups
 *
 *                                                                         *
 ***************************************************************************/

#include "QLog.h"
#include <QMenu>
#include <QString>
#include <QSettings>
#include "QExtAction.h"

namespace lpzrobots {
  
  QLog* QLog::instance = 0;

  QLog::QLog(QString applicationPath) :
    applicationPath(applicationPath) {
    if (instance != 0) {
      // another instance is already running!
      textLog("QLog: I have to quit because another instance was created!", LOG_ERROR);
      delete instance;
    }
    instance = this;
    createActions();
    readSettings();
  }
  
  QLog::~QLog() {
    writeSettings();
    instance = 0;
  }

  void QLog::textLog(QString log, LOG_LEVEL logLevel /*= LOG_VERBOSE*/) {
    QLog* instance = getInstance();
    switch (logLevel) {
      case LOG_ERROR: // always log errors
        emit instance->sig_textLog("<b><font color=red>"+log+"</color></b>"); // forward
        break;
      case LOG_WARNING:
        if (instance->warningOutput)
          emit instance->sig_textLog("<b><font color=#c16d00>"+log+"</color></b>"); // forward
        break;
      case LOG_VERBOSE:
        if (instance->verboseOutput)
          emit instance->sig_textLog(log); // forward
        break;
      case LOG_DEBUG:
        if (instance->debugOutput)
          emit instance->sig_textLog("<font color=#404040>"+log+"</color>"); // forward
        break;
      default:
        break;
    }
  }


  void QLog::logError(QString log) {
    textLog(log, LOG_ERROR);
  }

  void QLog::logWarning(QString log) {
    textLog(log, LOG_WARNING);
  }

  void QLog::logVerbose(QString log){
    textLog(log, LOG_VERBOSE);
  }

  void QLog::logDebug(QString log, QString sourceFile, int sourceLine, QString sourceFunction) {
    textLog(formatDebugSource(sourceFile, sourceLine, sourceFunction).append(log), LOG_DEBUG);
  }

  bool QLog::isLevel(LOG_LEVEL logLevel /*= LOG_VERBOSE*/) {
    switch (logLevel) {
      case LOG_ERROR: // always log errors
        return true;
        break;
      case LOG_WARNING:
        if (getInstance()->warningOutput)
          return true;
        break;
      case LOG_VERBOSE:
        if (getInstance()->verboseOutput)
          return true;
        break;
      case LOG_DEBUG:
        if (getInstance()->debugOutput)
          return true;
        break;
      default:
        break;
    }
    return false;
  }


  void QLog::sl_GUIEventHandler(int eventCode) {

    switch (eventCode) {
      case EVENT_SWITCH_WARNING:
        warningOutput = action_SwitchWarning->isChecked();
        emit sig_textLog("Set warning output to " + QString::number(warningOutput));
        break;
      case EVENT_SWITCH_VERBOSE:
        verboseOutput = action_SwitchVerbose->isChecked();
        emit sig_textLog("Set verbose output to " + QString::number(verboseOutput));
        break;
      case EVENT_SWITCH_DEBUG:
        debugOutput = action_SwitchDebug->isChecked();
        emit sig_textLog("Set debug output to " + QString::number(debugOutput));
        break;
      default:
        break;
    }
    writeSettings();
  }

  void QLog::createActions() {
    action_SwitchWarning = new QExtAction(EVENT_SWITCH_WARNING, (tr("&Warning log")), this);
    action_SwitchWarning->setCheckable(true);
    action_SwitchWarning->setStatusTip(tr("Enable/Disable verbosed output"));
    connect(action_SwitchWarning, SIGNAL(triggered(int)), this, SLOT(sl_GUIEventHandler(int)));

    action_SwitchVerbose = new QExtAction(EVENT_SWITCH_VERBOSE, (tr("&Verbose log")), this);
    action_SwitchVerbose->setCheckable(true);
    action_SwitchVerbose->setStatusTip(tr("Enable/Disable verbosed output"));
    connect(action_SwitchVerbose, SIGNAL(triggered(int)), this, SLOT(sl_GUIEventHandler(int)));

    action_SwitchDebug = new QExtAction(EVENT_SWITCH_DEBUG, (tr("&Debug log")), this);
    action_SwitchDebug->setCheckable(true);
    action_SwitchDebug->setStatusTip(tr("Enable/Disable debug output"));
    connect(action_SwitchDebug, SIGNAL(triggered(int)), this, SLOT(sl_GUIEventHandler(int)));
  }

  void QLog::addActionsToMenu(QMenu* menu) {
    menu->addAction(action_SwitchWarning);
    menu->addAction(action_SwitchVerbose);
    menu->addAction(action_SwitchDebug);
  }

  void QLog::readSettings() {
    QSettings settings(applicationPath + QString("log.settings"), QSettings::IniFormat);

    warningOutput = settings.value("warningOutput", true).toBool();
    verboseOutput = settings.value("verboseOutput", true).toBool();
    debugOutput = settings.value("debugOutput", false).toBool();
    action_SwitchWarning->setChecked(warningOutput);
    action_SwitchVerbose->setChecked(verboseOutput);
    action_SwitchDebug->setChecked(debugOutput);
  }

  void QLog::writeSettings() {
    QSettings settings(applicationPath + QString("log.settings"), QSettings::IniFormat);
    settings.setValue("warningOutput", warningOutput);
    settings.setValue("verboseOutput", verboseOutput);
    settings.setValue("debugOutput", debugOutput);
  }

  QString QLog::formatDebugSource(QString file, int line, QString sourceFunction) {
    if (sourceFunction.size()==0)
      return QString("<font color=#707070>[").append(file.mid(file.lastIndexOf('/')+1)).append(":").append(QString::number(line)).append("]</font>");
    else
      return QString("<font color=#707070>[").append(file.mid(file.lastIndexOf('/')+1)).append(":").append(QString::number(line)).append(":").append(sourceFunction).append("()]</font>");
  }


}
