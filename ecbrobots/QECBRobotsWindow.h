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
 *   Revision 1.23  2011-04-06 06:48:59  guettler
 *   - main window activates when hovering with mouse, enabling keyboard shortcuts without click onto the window
 *
 *   Revision 1.22  2011/04/04 09:25:03  guettler
 *   - loopStateLabel now updates each control step, displaying current status and control step
 *
 *   Revision 1.21  2011/03/25 22:53:07  guettler
 *   - autoload function did not allow changing the configurable values during the
 *     initialization phase of the loop, this is now supported, so
 *   - if you like to add configurable parameters which are used in
 *     QECBManager::start(..), just add them to globaldata, then the parameters can
 *     be changed before starting the control loop.
 *   - All other parameters of the ECBAgent and it's configurable childs (Robot, ECB,
 *     Controller, ...) are only configurable while the control loop is running (or paused).
 *
 *   Revision 1.20  2011/03/23 12:37:11  guettler
 *   - configurable childs are now intended
 *   - cleanup
 *
 *   Revision 1.19  2011/03/22 16:38:01  guettler
 *   - adpaptions to enhanced configurable and inspectable interface:
 *   - qconfigurable is now restarted if initialization of agents is finished
 *
 *   Revision 1.18  2011/03/21 17:32:19  guettler
 *   - adapted to enhanced configurable interface
 *   - support for configurable childs of a configurable
 *
 *   Revision 1.17  2011/02/04 13:00:50  wrabe
 *   - bugfix: Configurables are restored now when event "CommunicationStateWillChange" occurs
 *
 *   Revision 1.16  2011/01/28 12:15:37  guettler
 *   - restore of AutoSave File from a backup implemented
 *   - reset to original values, values AND bounds for Configurable implemented
 *   - reset to original values for tileWidgets implemented
 *
 *   Revision 1.15  2011/01/27 15:48:01  guettler
 *   - pause modus fixed
 *
 *   Revision 1.14  2011/01/24 16:58:25  guettler
 *   - QMessageDispatchServer is now informed when client app closes itself
 *   - QMessageDispatchWindow actually closes if client app closes itself
 *   - hint: this should late be
 *
 *   Revision 1.13  2011/01/24 14:17:39  guettler
 *   - new menu entry start/stop MatrixViz
 *
 *   Revision 1.12  2010/12/16 16:48:47  wrabe
 *   -integrated the statusbar
 *
 *   Revision 1.11  2010/12/15 11:10:26  wrabe
 *   -clear function for AutoSave File
 *
 *   Revision 1.10  2010/12/15 11:00:06  wrabe
 *   -load/save multiple ConfigurableStates from one file
 *   -All current ConfigurableStates can be stored and loaded now via menu
 *   -loading a ConfigurableState for one Configurable from a file containing multiple ConfigurableStates allows to choose one desired ConfigurableState
 *
 *   Revision 1.9  2010/12/14 11:11:06  guettler
 *   -preparations for global save functionality
 *
 *   Revision 1.8  2010/12/14 10:10:12  guettler
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
      void enterEvent(QEvent *event);

    public slots:
      void sl_textLog(QString s);
      void sl_GUIEventHandler(int eventCode);
      void sl_configurableChanged(QConfigurableWidget* sourceWidget);

    private slots:
      void sl_Close();
      void sl_ClearLogView();
      void sl_About();

      void sl_CommunicationStateWillChange(QECBCommunicator::ECBCommunicationState fromState, QECBCommunicator::ECBCommunicationState toState);
      void sl_CommunicationStateChanged(QECBCommunicator::ECBCommunicationState commState);
      void sl_saveCurrentConfigurableStatesToFile();
      void sl_loadCurrentConfigurableStatesFromFile();
      void sl_clearAutoSaveFile();
      void sl_restoreAutoSaveFile();
      void sl_statusLabelTimerExpired();
      void sl_updateLoopStateLabel();

    signals:
      void sig_quitClient();

    private:

      void printBuffer(QByteArray buffer);
      void createActions();
      void createMenus();
      void readSettings();
      void writeSettings();
      void sleep(ulong msecs);

      /**
       * recursive method to add a list of configurables to the given grid.
       * @param configList the list of configurables to add
       * @param grid the grid to add the configurables
       * @param configurableIndexMap the map where all configurables are indexed
       * @param configurableWidgetIndex optional, must be set at recursive calls
       * @param embeddingDepth optional, determines the embedding depth used for the
       *        widgets to add (configurable childs)
       * @return the number of widgets added
       */
      int addConfigurablesToGrid(ConfigList configList, QGridLayout* grid, QHash<QString, int>& configurableIndexMap, int configurableWidgetIndex = 0, int embeddingDepth = 0);

      QWidget* createConfigurableWidget();
      void updateConfigurableWidget();
      void restoreOriginalConfigurables();
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
      QAction *action_SaveConfigurableState;
      QAction *action_LoadConfigurableState;
      QAction *action_ClearAutoSaveFile;
      QAction *action_RestoreAutoSaveFile;
      QAction *action_Exit;
      // Help-Menu
      QAction *action_About;
      // Start/stop button
      QExtAction* action_StartLoop;
      QExtAction* action_RestartLoop;
      QExtAction* action_PauseLoop;
      QExtAction* action_StopLoop;
      QExtAction* action_StartStopGuiLogger;
      QExtAction* action_StartStopMatrixViz;

      QExtAction* action_SwitchWarning;
      QExtAction* action_SwitchVerbose;
      QExtAction* action_SwitchDebug;

      QWidget* configWidget;
      QScrollArea* scrollArea;
      QMap<QString, QConfigurableWidget*> configurableWidgetMap;
      QHash<QString, QDomElement> nodeConfigurableStateMap;

      QLabel* statusLabel;
      QTimer statusLabelTimer;
      QLabel* loopStateLabel;

      bool isClosed;

  };

} // namespace lpzrobots

#endif // __QECBROBOTSWINDOW_H
