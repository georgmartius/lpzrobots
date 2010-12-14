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
 *   Revision 1.8  2010-12-14 10:10:12  guettler
 *   -autoload/autosave now uses only one xml file
 *   -fixed getName of TileWidget which produced invisible widgets in xml files
 *
 *   Revision 1.7  2010/12/13 16:22:18  wrabe
 *   - autosave function rearranged
 *   - bugfixes
 *
 *   Revision 1.6  2010/12/09 17:00:08  wrabe
 *   - load / save function of ConfigurableState (configurable + GUI)
 *   - autoload / autosave function of ConfigurableState (configurable
 *     + GUI)
 *   - handling of equal Configurable names implemented for autoload
 *     and -save
 *   - bugfixing
 *
 *   Revision 1.5  2010/12/08 17:47:27  wrabe
 *   - bugfixing/introducing new feature:
 *   - folding of the ConfigurableWidgets now awailable
 *
 *   Revision 1.4  2010/11/26 12:22:37  guettler
 *   - Configurable interface now allows to set bounds of paramval and paramint
 *     * setting bounds for paramval and paramint is highly recommended (for QConfigurable (Qt GUI).
 *   - bugfixes
 *   - current development state of QConfigurable (Qt GUI)
 *
 *   Revision 1.3  2010/11/19 15:15:00  guettler
 *   - new QLog feature
 *   - bugfixes
 *   - FT232Manager is now in lpzrobots namespace
 *   - some cleanups
 *
 *   Revision 1.2  2010/11/11 15:34:59  wrabe
 *   - some extensions for QMessageClient (e.g. quitServer())
 *   - fixed some includes
 *
 *   Revision 1.1  2010/11/10 09:32:00  guettler
 *   - port to Qt part 1
 *                                                *
 *                                                                         *
 ***************************************************************************/

#ifndef __QECBROBOTSWINDOW_H
#define __QECBROBOTSWINDOW_H

#include <QtGui>
#include <QMainWindow>
#include <QTextEdit>
#include <QTimer>
#include <QStyleOptionProgressBarV2>
#include <qdom.h>
#include "QExtAction.h"
#include <QHash>

#include "types.h"
#include "QLogViewWidget.h"
#include "QAbstractMessageClient.h"
#include "QECBManager.h"
#include "QECBCommunicator.h"
#include "QConfigurableWidget.h"

namespace lpzrobots {

  class QECBRobotsWindow : public QMainWindow {

    Q_OBJECT

    public:
      QECBRobotsWindow(QString applicationPath, QECBManager* manager);

    protected:
      void closeEvent(QCloseEvent *event);

    public slots:
      void sl_textLog(QString s);
      void sl_GUIEventHandler(int eventCode);


    private slots:
      void sl_Close();
      void sl_ClearLogView();
      void sl_About();

      void sl_CommunicationStateWillChange(QECBCommunicator::ECBCommunicationState commState);
      void sl_CommunicationStateChanged(QECBCommunicator::ECBCommunicationState commState);

    private:

      void printBuffer(QByteArray buffer);
      void createActions();
      void createMenus();
      void readSettings();
      void writeSettings();
      void sleep(ulong msecs);
      QWidget* createConfigurableWidget();
      void updateConfigurableWidget();
      void bookmarkConfigurableStates();
      void recallConfigurableStates();
      void autostoreConfigurableStates();
      void autoloadConfigurableStates();

      enum GUI_EVENT {
        EVENT_SWITCH_WARNING, EVENT_SWITCH_VERBOSE, EVENT_SWITCH_DEBUG
      };

      QString applicationPath;

      QECBManager* ecbManager;
      QGlobalData* globalData;

      QTabWidget *tabWidget;
      QLogViewWidget *logView;

      QMenu *fileMenu;
      QMenu *loopControlMenu;
      QMenu *additionalsMenu;
      QMenu *settingsMenu;
      QMenu *helpMenu;

      // --- Actions ----------------------------------
      // File-Menu
      QAction *action_Exit;
      // Help-Menu
      QAction *action_About;
      // Start/stop button
      QExtAction* action_StartLoop;
      QExtAction* action_RestartLoop;
      QExtAction* action_PauseLoop;
      QExtAction* action_StopLoop;
      QExtAction* action_StartStopGuiLogger;

      QExtAction* action_SwitchWarning;
      QExtAction* action_SwitchVerbose;
      QExtAction* action_SwitchDebug;

      QWidget* configWidget;
      QScrollArea* scrollArea;
      QList<QConfigurableWidget*> configurableWidgetList;
      QHash<QString,QDomElement> nodeConfigurableStateMap;

      bool isClosed;

  };

} // namespace lpzrobots

#endif // __QECBROBOTSWINDOW_H
