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

#ifndef __QLOG_H_
#define __QLOG_H_

#include <QObject>

class QMenu;
class QString;

#include <linux/kernel.h>

#define QLOG(String, Level)\
  if(QLog::isLevel(Level)) QLog::textLog(String, Level);

// note: always log errors
#define QLogError(errorString)\
    QLog::textLog(errorString, QLog::LOG_ERROR);

#define QLogWarning(warningString)\
  if(QLog::isLevelWarning()) QLog::textLog(warningString, QLog::LOG_WARNING);

#define QLogVerbose(verboseString)\
  if(QLog::isLevelVerbose()) QLog::textLog(verboseString, QLog::LOG_VERBOSE);

#define QLogDebug(debugString)\
  if(QLog::isLevelDebug()) QLog::textLog(QLog::formatDebugSource(__FILE__, __LINE__, __FUNCTION__).append(debugString), QLog::LOG_DEBUG);

namespace lpzrobots {
  
  class QExtAction;

  /**
   * Singleton class. Should be instantiated by the object which is responsible for
   * displaying the log.
   * TODO: log to file
   */
  class QLog : public QObject {
    Q_OBJECT

    public:
      QLog(QString applicationPath = "./");

      virtual ~QLog();

      enum LOG_LEVEL {
        LOG_ERROR, LOG_WARNING, LOG_VERBOSE, LOG_DEBUG,
      };

      /**
       * @param given QString will be forwarded to the log window (via sig_textLog)
       * @param determines which type of log is made (error, warningOutput, verboseOutput, debug)
       */
      static void textLog(QString log, LOG_LEVEL logLevel = LOG_VERBOSE);

      static QString formatDebugSource(QString file, int line, QString sourceFunction);

      /**
       * Determines if the given log level is actual active.
       * Inactive log levels are neither not logged to file nor forwarded.
       * @param logType
       * @return true or false
       */
      static bool isLevel(LOG_LEVEL logLevel = LOG_VERBOSE);
      static inline bool isLevelWarning() {
        return getInstance()->warningOutput;
      }
      static inline bool isLevelVerbose() {
        return getInstance()->verboseOutput;
      }
      static inline bool isLevelDebug() {
        return getInstance()->debugOutput;
      }

      /**
       * In order to toggle the log outputs this method adds all needed actions to the given menu.
       * So you don't need to take care about the actions and their signals
       * @param menu
       */
      void addActionsToMenu(QMenu* menu);

    signals:
      /**
       * if emitted, given QString will be forwarded to the log window which instantiates this class
       */
      void sig_textLog(QString log);

    protected:
      bool warningOutput;
      bool verboseOutput;
      bool debugOutput;

      static QLog* instance; // Singleton instance

      QString applicationPath;

      QExtAction* action_SwitchWarning;
      QExtAction* action_SwitchVerbose;
      QExtAction* action_SwitchDebug;

      enum ACTION_EVENT {
        EVENT_SWITCH_WARNING, EVENT_SWITCH_VERBOSE, EVENT_SWITCH_DEBUG
      };

      /**
       * Returns the singleton instance of this class.
       * If not exists, it will be created (but logs are not forwarded yet).
       * @return
       */
      static inline QLog* getInstance() {
        if (instance == 0)
          instance = new QLog();
        return instance;
      }

      void createActions();

      void readSettings();
      void writeSettings();

    protected slots:

      void sl_GUIEventHandler(int eventCode);

    protected:
      // log critical?
      // log info?
      // log debugDetail?
      /**
       * @deprecated
       * @param log
       */
      static void logError(QString log);
      /**
       * @deprecated
       * @param log
       */
      static void logWarning(QString log);
      /**
       * @deprecated
       * @param log
       */
      static void logVerbose(QString log);
      /**
       * @deprecated
       * @param log
       */
      static void logDebug(QString log, QString sourceFile, int sourceLine, QString sourceFunction);

  };

}

#endif /* __QLOG_H_ */
