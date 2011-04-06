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
 *   Revision 1.32  2011-04-06 06:48:59  guettler
 *   - main window activates when hovering with mouse, enabling keyboard shortcuts without click onto the window
 *
 *   Revision 1.31  2011/04/04 09:25:03  guettler
 *   - loopStateLabel now updates each control step, displaying current status and control step
 *
 *   Revision 1.30  2011/03/25 22:53:07  guettler
 *   - autoload function did not allow changing the configurable values during the
 *     initialization phase of the loop, this is now supported, so
 *   - if you like to add configurable parameters which are used in
 *     QECBManager::start(..), just add them to globaldata, then the parameters can
 *     be changed before starting the control loop.
 *   - All other parameters of the ECBAgent and it's configurable childs (Robot, ECB,
 *     Controller, ...) are only configurable while the control loop is running (or paused).
 *
 *   Revision 1.29  2011/03/25 20:59:19  guettler
 *   - fixed the problem that when a has configurable changed and the widgets were
 *     rebuild the autosave function did not funtion properly anymore
 *
 *   Revision 1.28  2011/03/23 12:37:11  guettler
 *   - configurable childs are now intended
 *   - cleanup
 *
 *   Revision 1.27  2011/03/22 16:38:01  guettler
 *   - adpaptions to enhanced configurable and inspectable interface:
 *   - qconfigurable is now restarted if initialization of agents is finished
 *
 *   Revision 1.26  2011/03/21 17:32:19  guettler
 *   - adapted to enhanced configurable interface
 *   - support for configurable childs of a configurable
 *
 *   Revision 1.25  2011/02/11 12:12:01  guettler
 *   - UI: some seperators added
 *
 *   Revision 1.24  2011/02/04 13:00:50  wrabe
 *   - bugfix: Configurables are restored now when event "CommunicationStateWillChange" occurs
 *
 *   Revision 1.23  2011/01/28 12:15:37  guettler
 *   - restore of AutoSave File from a backup implemented
 *   - reset to original values, values AND bounds for Configurable implemented
 *   - reset to original values for tileWidgets implemented
 *
 *   Revision 1.22  2011/01/28 11:32:12  guettler
 *   - original values are written back to the Configurable instances if the QConfigurable interface is restarted
 *
 *   Revision 1.21  2011/01/27 15:48:01  guettler
 *   - pause modus fixed
 *
 *   Revision 1.20  2011/01/24 18:40:48  guettler
 *   - autosave functionality now stores only values, bounds and descriptions of
 *   parameters if they differ from their original values
 *
 *   Revision 1.19  2011/01/24 16:58:25  guettler
 *   - QMessageDispatchServer is now informed when client app closes itself
 *   - QMessageDispatchWindow actually closes if client app closes itself
 *   - hint: this should late be
 *
 *   Revision 1.18  2011/01/24 14:17:39  guettler
 *   - new menu entry start/stop MatrixViz
 *
 *   Revision 1.17  2010/12/16 18:37:40  wrabe
 *   -added several tooltips
 *   -corrected sentences, notation, syntax for improved informational value
 *   -bugfix: if in collapsed mode, all tiles were stored as invisible
 *   -cosmetic ui changes
 *   -other minor things
 *
 *   Revision 1.16  2010/12/16 16:48:47  wrabe
 *   -integrated the statusbar
 *
 *   Revision 1.15  2010/12/15 17:26:28  wrabe
 *   - number of colums for tileWidgets and width of tileWidgets can
 *   now be changed (independently for each Configurable)
 *   - bugfixes
 *
 *   Revision 1.14  2010/12/15 11:19:30  wrabe
 *   -some shortcuts rearranged
 *   -last active tab is remarked in settings
 *
 *   Revision 1.13  2010/12/15 11:10:26  wrabe
 *   -clear function for AutoSave File
 *
 *   Revision 1.12  2010/12/15 11:00:06  wrabe
 *   -load/save multiple ConfigurableStates from one file
 *   -All current ConfigurableStates can be stored and loaded now via menu
 *   -loading a ConfigurableState for one Configurable from a file containing multiple ConfigurableStates allows to choose one desired ConfigurableState
 *
 *   Revision 1.11  2010/12/14 11:11:06  guettler
 *   -preparations for global save functionality
 *
 *   Revision 1.10  2010/12/14 10:10:12  guettler
 *   -autoload/autosave now uses only one xml file
 *   -fixed getName of TileWidget which produced invisible widgets in xml files
 *
 *   Revision 1.9  2010/12/13 16:22:18  wrabe
 *   - autosave function rearranged
 *   - bugfixes
 *
 *   Revision 1.8  2010/12/09 17:00:08  wrabe
 *   - load / save function of ConfigurableState (configurable + GUI)
 *   - autoload / autosave function of ConfigurableState (configurable
 *     + GUI)
 *   - handling of equal Configurable names implemented for autoload
 *     and -save
 *   - bugfixing
 *
 *   Revision 1.7  2010/12/08 17:47:27  wrabe
 *   - bugfixing/introducing new feature:
 *   - folding of the ConfigurableWidgets now awailable
 *
 *   Revision 1.6  2010/11/30 17:21:20  wrabe
 *   - bugfix
 *
 *   Revision 1.5  2010/11/28 20:22:51  wrabe
 *   - the centralWidget is now the QTabWidget instead of a separate QWidget containing a QTabWidget
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
 *                                              *
 *                                                                         *
 ***************************************************************************/

#include "QECBRobotsWindow.h"
#include "QConfigurableLoadSaveDialog.h"
#include  <QScrollArea>

namespace lpzrobots {

  QECBRobotsWindow::QECBRobotsWindow(QString applicationPath, QECBManager* manager) :
    configWidget(0), isClosed(false) {
    this->applicationPath = applicationPath;
    this->setWindowTitle("ECB_Robot-Application V2.1");
    loopStateLabel = new QLabel(statusBar());
    statusLabel = new QLabel(statusBar());

    statusBar()->addWidget(statusLabel, 1);
    statusBar()->addWidget(loopStateLabel);
    connect(&statusLabelTimer, SIGNAL(timeout()), this, SLOT(sl_statusLabelTimerExpired()));
    loopStateLabel->setText("STOPPED");


    autoloadConfigurableStates();

    logView = new QLogViewWidget();

    logView->appendLogViewText("ApplicationPath='" + applicationPath + "'");

    // Layout:
    //------------------------------------------------------
    //    QGridLayout *grid = new QGridLayout();
    //    QWidget *mainpanel = new QWidget();
    //    mainpanel->setLayout(grid);
    //    setCentralWidget(mainpanel);


    tabWidget = new QTabWidget;
    tabWidget->addTab(logView, tr("Report"));
    setCentralWidget(tabWidget);

    this->ecbManager = manager;
    this->globalData = &(manager->getGlobalData());
    connect(globalData, SIGNAL(sig_textLog(QString)), this, SLOT(sl_textLog(QString)));
    connect(ecbManager, SIGNAL(sig_communicationStateWillChange(QECBCommunicator::ECBCommunicationState, QECBCommunicator::ECBCommunicationState)), this,
        SLOT(sl_CommunicationStateWillChange(QECBCommunicator::ECBCommunicationState, QECBCommunicator::ECBCommunicationState)));
    connect(ecbManager, SIGNAL(sig_communicationStateChanged(QECBCommunicator::ECBCommunicationState)), this,
        SLOT(sl_CommunicationStateChanged(QECBCommunicator::ECBCommunicationState)));
    connect(globalData->comm, SIGNAL(sig_quitServer()), this, SLOT(sl_Close()));
    connect(this, SIGNAL(sig_quitClient()), globalData->comm, SLOT(sl_quitClient()));
    connect(globalData->comm, SIGNAL(sig_stepDone()), this, SLOT(sl_updateLoopStateLabel()));


    createActions();
    createMenus();

    readSettings();

    // start the QECBManager (1st stage loop, 2nd stage loop is started by QECBCommunicator)
    this->ecbManager->initialize();

    sl_updateLoopStateLabel();
    //updateConfigurableWidget();
  }

  void QECBRobotsWindow::createActions() {
    action_SaveConfigurableState = new QAction((tr("Save ConfigurableStates ...")), this);
    action_SaveConfigurableState->setShortcut(tr("F7"));
    action_SaveConfigurableState->setStatusTip(tr("Saves one or more currently used ConfigurableStates to an XML file."));
    connect(action_SaveConfigurableState, SIGNAL(triggered()), this, SLOT(sl_saveCurrentConfigurableStatesToFile()));

    action_LoadConfigurableState = new QAction((tr("Load ConfigurableStates ...")), this);
    action_LoadConfigurableState->setShortcut(tr("F8"));
    action_LoadConfigurableState->setStatusTip(tr("Loads one or more ConfigurableStates from an XML file."));
    connect(action_LoadConfigurableState, SIGNAL(triggered()), this, SLOT(sl_loadCurrentConfigurableStatesFromFile()));

    action_ClearAutoSaveFile = new QAction((tr("Clear AutoSave File")), this);
    action_ClearAutoSaveFile->setStatusTip(tr("Removes all currently not used saved ConfigurableStates from the AutoSave File (autosave_QConfigurable.xml)."));
    connect(action_ClearAutoSaveFile, SIGNAL(triggered()), this, SLOT(sl_clearAutoSaveFile()));

    action_RestoreAutoSaveFile = new QAction((tr("Restore AutoSave File")), this);
    action_RestoreAutoSaveFile->setStatusTip(tr("Restores the AutoSave File from a backup created at the start of this program and restores the ConfigurableStates (autosave_QConfigurable_backup.xml)."));
    connect(action_RestoreAutoSaveFile, SIGNAL(triggered()), this, SLOT(sl_restoreAutoSaveFile()));

    action_Exit = new QAction(tr("&Quit"), this);
    action_Exit->setShortcut(tr("Ctrl+Q"));
    action_Exit->setStatusTip(tr("Closes the application."));
    connect(action_Exit, SIGNAL(triggered()), this, SLOT(close()));

    action_StartLoop = new QExtAction(QECBManager::EVENT_START_LOOP, (tr("&Start loop")), this);
    action_StartLoop->setShortcut(tr("CTRL+S"));
    action_StartLoop->setStatusTip(tr("Starts the control loop."));
    connect(action_StartLoop, SIGNAL(triggered(int)), ecbManager, SLOT(sl_GUIEventHandler(int)));

    action_RestartLoop = new QExtAction(QECBManager::EVENT_RESTART_LOOP, (tr("&Restart loop")), this);
    action_RestartLoop->setShortcut(tr("CTRL+R"));
    action_RestartLoop->setStatusTip(tr("Stops and Restarts the control loop."));
    connect(action_RestartLoop, SIGNAL(triggered(int)), ecbManager, SLOT(sl_GUIEventHandler(int)));

    action_PauseLoop = new QExtAction(QECBManager::EVENT_PAUSE_LOOP, (tr("&Pause/Unpause loop")), this);
    action_PauseLoop->setShortcut(tr("SPACE"));
    action_PauseLoop->setStatusTip(tr("Pauses or Unpauses the control loop."));
    connect(action_PauseLoop, SIGNAL(triggered(int)), ecbManager, SLOT(sl_GUIEventHandler(int)));

    action_StopLoop = new QExtAction(QECBManager::EVENT_STOP_LOOP, (tr("&Stop loop")), this);
    action_StopLoop->setShortcut(tr("CTRL+S"));
    action_StopLoop->setStatusTip(tr("Stops the control loop."));
    connect(action_StopLoop, SIGNAL(triggered(int)), ecbManager, SLOT(sl_GUIEventHandler(int)));

    action_StartStopGuiLogger = new QExtAction(QECBManager::EVENT_START_GUILOGGER, (tr("Start/Stop &Guilogger")), this);
    action_StartStopGuiLogger->setShortcut(tr("CTRL+G"));
    action_StartStopGuiLogger->setStatusTip(tr("Starts/Stops the Guilogger(s)."));
    connect(action_StartStopGuiLogger, SIGNAL(triggered(int)), ecbManager, SLOT(sl_GUIEventHandler(int)));

    action_StartStopMatrixViz = new QExtAction(QECBManager::EVENT_START_MATRIXVIZ, (tr("Start/Stop &MatrixViz")), this);
    action_StartStopMatrixViz->setShortcut(tr("CTRL+M"));
    action_StartStopMatrixViz->setStatusTip(tr("Starts/Stops the MatrixViz(s)."));
    connect(action_StartStopMatrixViz, SIGNAL(triggered(int)), ecbManager, SLOT(sl_GUIEventHandler(int)));

    action_SwitchWarning = new QExtAction(EVENT_SWITCH_WARNING, (tr("&Warning log")), this);
    action_SwitchWarning->setCheckable(true);
    action_SwitchWarning->setStatusTip(tr("Enables/Disables the warning output."));
    connect(action_SwitchWarning, SIGNAL(triggered(int)), this, SLOT(sl_GUIEventHandler(int)));
    action_SwitchWarning->setChecked(true);

    action_SwitchVerbose = new QExtAction(EVENT_SWITCH_VERBOSE, (tr("&Verbose log")), this);
    action_SwitchVerbose->setCheckable(true);
    action_SwitchVerbose->setStatusTip(tr("Enables/Disables the verbose output."));
    connect(action_SwitchVerbose, SIGNAL(triggered(int)), this, SLOT(sl_GUIEventHandler(int)));
    action_SwitchVerbose->setChecked(true);

    action_SwitchDebug = new QExtAction(EVENT_SWITCH_DEBUG, (tr("&Debug log")), this);
    action_SwitchDebug->setCheckable(true);
    action_SwitchDebug->setStatusTip(tr("Enables/Disables the debug output."));
    connect(action_SwitchDebug, SIGNAL(triggered(int)), this, SLOT(sl_GUIEventHandler(int)));
    action_SwitchDebug->setChecked(false);

    sl_CommunicationStateChanged(QECBCommunicator::STATE_STOPPED);
    sl_GUIEventHandler(EVENT_SWITCH_WARNING);
    sl_GUIEventHandler(EVENT_SWITCH_VERBOSE);
    sl_GUIEventHandler(EVENT_SWITCH_DEBUG);

    // Actions About
    action_About = new QAction(tr("About ..."), this);
    action_About->setStatusTip(tr("Shows the application's About box."));
    connect(action_About, SIGNAL(triggered()), this, SLOT(sl_About()));

  }

  void QECBRobotsWindow::createMenus() {
    //delete this->menuBar();
    this->menuBar()->clear();

    fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(action_SaveConfigurableState);
    fileMenu->addAction(action_LoadConfigurableState);
    fileMenu->addSeparator();
    fileMenu->addAction(action_ClearAutoSaveFile);
    fileMenu->addAction(action_RestoreAutoSaveFile);
    fileMenu->addSeparator();
    fileMenu->addAction(action_Exit);

    loopControlMenu = menuBar()->addMenu(tr("&LoopControl"));
    loopControlMenu->addAction(action_StartLoop);
    // loopControlMenu->addSeparator();
    loopControlMenu->addAction(action_RestartLoop);
    loopControlMenu->addAction(action_PauseLoop);
    loopControlMenu->addAction(action_StopLoop);

    additionalsMenu = menuBar()->addMenu(tr("&Additionals"));
    additionalsMenu->addAction(action_StartStopGuiLogger);
    additionalsMenu->addAction(action_StartStopMatrixViz);

    settingsMenu = menuBar()->addMenu(tr("&Settings"));
    settingsMenu->addAction(action_SwitchWarning);
    settingsMenu->addAction(action_SwitchVerbose);
    settingsMenu->addAction(action_SwitchDebug);

    helpMenu = menuBar()->addMenu(tr("&Help"));
    helpMenu->addAction(action_About);
  }

  void QECBRobotsWindow::readSettings() {
    QSettings settings(applicationPath + QString("ecbrobots.settings"), QSettings::IniFormat);
    QPoint pos = settings.value("pos", QPoint(200, 200)).toPoint();
    QSize size = settings.value("size", QSize(400, 400)).toSize();
    resize(size);
    move(pos);
    bool warning = settings.value("warningOutput").toBool();
    bool verbose = settings.value("verboseOutput").toBool();
    bool debug = settings.value("debugOutput").toBool();
    action_SwitchWarning->setChecked(warning);
    action_SwitchVerbose->setChecked(verbose);
    action_SwitchDebug->setChecked(debug);
    globalData->warningOutput = warning;
    globalData->verboseOutput = verbose;
    globalData->debugOutput = debug;
    tabWidget->setCurrentIndex(settings.value("activeTabIndex", 1).toInt());
  }

  void QECBRobotsWindow::writeSettings() {
    QSettings settings(applicationPath + QString("ecbrobots.settings"), QSettings::IniFormat);
    settings.setValue("pos", pos());
    settings.setValue("size", size());
    settings.setValue("warningOutput", globalData->warningOutput);
    settings.setValue("verboseOutput", globalData->verboseOutput);
    settings.setValue("debugOutput", globalData->debugOutput);
    settings.setValue("activeTabIndex", tabWidget->currentIndex());
  }

  void QECBRobotsWindow::closeEvent(QCloseEvent *event) {
    // Vermutung: es bleibt ein Object erhalten, sodass dadurch dieses
    // Fenster nicht beendet.
    // Folge: fehler in der Autosave-Funktion
    // TODO: behebe es ...

    if (!isClosed) {
      //      QString text = "QECBRobotsWindow::closeEvent(";
      //      foreach(QConfigurableWidget* confWidget, configurableWidgetList)
      //        {
      //          text.append(confWidget->getName() + ",");
      //        }
      //      text.append(")");
      //      QMessageBox msgBox;
      //      msgBox.setText(text);
      //      msgBox.exec();

      writeSettings();
      bookmarkConfigurableStates();
      autostoreConfigurableStates();
      isClosed = true;
      emit sig_quitClient();
    }
    event->accept();
  }

  void QECBRobotsWindow::sl_textLog(QString sText) {
    statusLabelTimer.stop();
    statusLabel->setText(sText);
    statusLabelTimer.start(5000);
    logView->appendLogViewText(sText);
  }

  void QECBRobotsWindow::sl_statusLabelTimerExpired() {
    statusLabel->setText("");
  }

  void QECBRobotsWindow::sl_ClearLogView() {
    logView->clearLogViewText();
  }

  void QECBRobotsWindow::sl_Close() {
    close();
  }

  void QECBRobotsWindow::sl_About() {
    QMessageBox::about(this, tr("About the Application"), tr(
        "ECB_Robot-Application V2.0, a tool to connect real robots (containing an ECB) onto a neuro-controller located on a standard desktop PC."));
  }

  void QECBRobotsWindow::sl_CommunicationStateWillChange(QECBCommunicator::ECBCommunicationState fromState, QECBCommunicator::ECBCommunicationState toState) {
    switch (toState) {
      case QECBCommunicator::STATE_PAUSED: //!< state which indicates that all actions are paused
        break;
      case QECBCommunicator::STATE_RUNNING: //!< state which indicates that the loop is running
        if (fromState == QECBCommunicator::STATE_STOPPED)
          bookmarkConfigurableStates();
          break; // do not restore configurables now because globalData would not be right configured in initializing phase
      case QECBCommunicator::STATE_STOPPED: //!< state which indicates that all actions are stopped, quitted and leaved. Bye bye.
      default:
        // bookmark now their state, because the Configurable instances in global->configs maybe deleted after them.
        bookmarkConfigurableStates();
        restoreOriginalConfigurables();
        break;
    }
  }

  void QECBRobotsWindow::sl_CommunicationStateChanged(QECBCommunicator::ECBCommunicationState commState) {
    switch (commState) {
      case QECBCommunicator::STATE_RUNNING: //!< state which indicates that the loop is running
        action_StartLoop->setEnabled(false);
        action_RestartLoop->setEnabled(true);
        action_PauseLoop->setEnabled(true);
        action_StopLoop->setEnabled(true);
        globalData->textLog("STATE: Running");
        restoreOriginalConfigurables();
        updateConfigurableWidget();

        break;
      case QECBCommunicator::STATE_PAUSED: //!< state which indicates that all actions are paused
        action_StartLoop->setEnabled(false);
        action_RestartLoop->setEnabled(true);
        action_PauseLoop->setEnabled(true);
        action_StopLoop->setEnabled(true);
        globalData->textLog("STATE: Paused");
        break;
      case QECBCommunicator::STATE_STOPPED: //!< state which indicates that all actions are stopped, quitted and leaved. Bye bye.
      default:
        action_StartLoop->setEnabled(true);
        action_RestartLoop->setEnabled(false);
        action_PauseLoop->setEnabled(false);
        action_StopLoop->setEnabled(false);
        globalData->textLog("STATE: Stopped");
        updateConfigurableWidget();
        break;
    }
    sl_updateLoopStateLabel();
  }

  void QECBRobotsWindow::sl_GUIEventHandler(int eventCode) {
    switch (eventCode) {
      case EVENT_SWITCH_WARNING:
        globalData->warningOutput = action_SwitchWarning->isChecked();
        globalData->textLog("Set warning output to " + QString::number(globalData->warningOutput));
        break;
      case EVENT_SWITCH_VERBOSE:
        globalData->verboseOutput = action_SwitchVerbose->isChecked();
        globalData->textLog("Set verbose output to " + QString::number(globalData->verboseOutput));
        break;
      case EVENT_SWITCH_DEBUG:
        globalData->debugOutput = action_SwitchDebug->isChecked();
        globalData->textLog("Set debug output to " + QString::number(globalData->debugOutput));
        break;
      default:
        break;
    }
  }

  void QECBRobotsWindow::restoreOriginalConfigurables() {
    // manual delete so they can restore the original parameters of the Configurables
    foreach(QConfigurableWidget* configWidget, configurableWidgetMap) {
      configWidget->sl_resetToOriginalValuesAndBounds();
    }
  }

  void QECBRobotsWindow::updateConfigurableWidget() {
    configurableWidgetMap.clear();
    int index = tabWidget->currentIndex();
    tabWidget->removeTab(1);
    tabWidget->insertTab(1, createConfigurableWidget(), "Configurables");
    tabWidget->setCurrentIndex(index);
  }

  int QECBRobotsWindow::addConfigurablesToGrid(ConfigList configList, QGridLayout* grid, QHash<QString, int>& configurableIndexMap, int configurableWidgetIndex /*= 0*/, int embeddingDepth) {
    FOREACH(ConfigList, configList, config) {
      int maxColumns = 20;
      QString name = QString((*config)->getName().c_str());
      int index = 0;
      if (configurableIndexMap.contains(name)) {
        index = configurableIndexMap[name] + 1;
      }
      configurableIndexMap[name] = index;
      QConfigurableWidget* confWidget = new QConfigurableWidget(*config, configurableIndexMap[name]);
      for (int col=0; col < embeddingDepth*2; col++) {
        QFrame* placeHolder = new QFrame;
        if (col%2)
          placeHolder->setFixedSize(10,10);
        else
          placeHolder->setFixedSize(8,20);
        placeHolder->setFrameStyle(QFrame::StyledPanel | QFrame::Raised);
        placeHolder->setAttribute(Qt::WA_DeleteOnClose);
        grid->addWidget(placeHolder, configurableWidgetIndex, col, Qt::AlignVCenter);//, i++, 0, Qt::AlignJustify);
      }
      if (embeddingDepth>0)
        grid->addWidget(confWidget, configurableWidgetIndex++, embeddingDepth*2, 1, maxColumns-embeddingDepth*2, Qt::AlignTop);//, i++, 0, Qt::AlignJustify);
      else
        grid->addWidget(confWidget, configurableWidgetIndex++, 0, 1, maxColumns, Qt::AlignTop);//, i++, 0, Qt::AlignJustify);
      configurableWidgetMap.insert(confWidget->getName(), confWidget);
      connect(confWidget, SIGNAL(sig_configurableChanged(QConfigurableWidget*)), this, SLOT(sl_configurableChanged(QConfigurableWidget*)));
      configurableWidgetIndex= addConfigurablesToGrid((*config)->getConfigurables(), grid, configurableIndexMap, configurableWidgetIndex, embeddingDepth+1);
    }
    return configurableWidgetIndex;
  }

  QWidget* QECBRobotsWindow::createConfigurableWidget() {
    if (configWidget != 0)
      delete configWidget;
    configWidget = new QWidget(); // containing some QGroupBoxes (QConfigurableWidget´s)
    QGridLayout* grid = new QGridLayout();
    grid->setSizeConstraint(QLayout::SetFixedSize);
    configWidget->setLayout(grid);
    QHash<QString, int> configurableIndexMap;
    int numberWidgetsAdded = addConfigurablesToGrid(globalData->configs, grid, configurableIndexMap);
    recallConfigurableStates(); // autoload function
    grid->setRowStretch(numberWidgetsAdded, 100);
    scrollArea = new QScrollArea();
    //scrollArea->setBackgroundRole(QPalette::Dark);
    scrollArea->setWidget(configWidget);
    scrollArea->setToolTip(QString::number(numberWidgetsAdded) + " Configurables");
    return scrollArea;
  }

  // is called when a configurable or one of their childs has been changed
  void QECBRobotsWindow::sl_configurableChanged(QConfigurableWidget* sourceWidget) {
    // simplest way: just recreate entire scrollarea
    bookmarkConfigurableStates();
    restoreOriginalConfigurables();
    updateConfigurableWidget();
  }

  void QECBRobotsWindow::autostoreConfigurableStates() {
    QString pathApplication = QCoreApplication::applicationDirPath();
    QString preferredFileName = pathApplication + "/autosave_QConfigurable.xml";
    QDomDocument doc("ConfigurableStateTypeDefinition");
    // <ConfigurableStates>
    QDomElement nodeConfigurableStates = doc.createElement("ConfigurableStates");
    doc.appendChild(nodeConfigurableStates);

    foreach (QDomElement entry, nodeConfigurableStateMap)
        nodeConfigurableStates.appendChild(entry);
    QFile file(preferredFileName);
    if (!file.open(QIODevice::WriteOnly))
      return;
    QTextStream ts(&file);
    ts << doc.toString();
    file.close();
  }

  void QECBRobotsWindow::autoloadConfigurableStates() {
    QString pathApplication = QCoreApplication::applicationDirPath();
    QString preferredFileName = pathApplication + "/autosave_QConfigurable.xml";
    // first backup autosave file
    QFile::remove(pathApplication + "/autosave_QConfigurable_backup.xml");
    QFile::copy(preferredFileName, pathApplication + "/autosave_QConfigurable_backup.xml");
    QFile file(preferredFileName);
    if (!file.open(QIODevice::ReadOnly))
      return;
    QDomDocument doc("ConfigurableStateTypeDefinition");
    if (!doc.setContent(&file)) {
      file.close();
      return;
    }
    file.close();

    // put all ConfigurableStates into the nodeConfigurableStateMap
    QDomElement qde_configurableStates = doc.documentElement();
    if (qde_configurableStates.tagName() != "ConfigurableStates")
      return;

    QDomNodeList nodeList = qde_configurableStates.elementsByTagName("ConfigurableState");
    for (int index = 0; index < nodeList.size(); index++) {
      QDomElement nodeConfigurableState = nodeList.item(index).toElement();
      QString name = nodeConfigurableState.attribute("name", "DefaultName");
      nodeConfigurableStateMap.insert(name, nodeConfigurableState);
    }
  }

  void QECBRobotsWindow::sl_restoreAutoSaveFile() {
    QString pathApplication = QCoreApplication::applicationDirPath();
    QString preferredFileName = pathApplication + "/autosave_QConfigurable.xml";
    // restore autosave file from backup
    QFile::remove(pathApplication + "/autosave_QConfigurable.xml");
    QFile::copy(pathApplication + "/autosave_QConfigurable_backup.xml", pathApplication + "/autosave_QConfigurable.xml");
    // load restored file
    nodeConfigurableStateMap.clear();
    autoloadConfigurableStates();
    recallConfigurableStates();
  }

  void QECBRobotsWindow::recallConfigurableStates() {
    foreach(QConfigurableWidget* confWidget, configurableWidgetMap)
      {
        QString configName = confWidget->getName();
        if (!nodeConfigurableStateMap.contains(configName) && confWidget->getNameIndex() != 0) {
          // try to load from another instance, the first one of a Configurable with the same name
          configName = QString(confWidget->getConfigurable()->getName().c_str()) + "_0";
        }
        if (nodeConfigurableStateMap.contains(configName)) {
          confWidget->fromXml(nodeConfigurableStateMap.value(configName), true);
        }
      }
  }

  void QECBRobotsWindow::bookmarkConfigurableStates() {
    // if some older ones exist, just overwrite them
    foreach(QConfigurableWidget* confWidget, configurableWidgetMap)
        nodeConfigurableStateMap.insert(confWidget->getName(), confWidget->toXml(false, true));
  }

  void QECBRobotsWindow::sl_loadCurrentConfigurableStatesFromFile() {
    QFileDialog* fileDialog = new QFileDialog();
    fileDialog->setWindowTitle("Select the XML file containing the ConfigurableState(s)");
    fileDialog->setAcceptMode(QFileDialog::AcceptOpen);
    fileDialog->setFileMode(QFileDialog::ExistingFile);
    QString pathApplication = QCoreApplication::applicationDirPath() + "/*.xml";
    fileDialog->selectFile(pathApplication);
    fileDialog->setNameFilter(tr("Xml (*.xml)"));
    if (fileDialog->exec() == QDialog::Accepted) {
      QFile file(fileDialog->selectedFiles().at(0));
      if (!file.open(QIODevice::ReadOnly))
        return;

      QDomDocument doc("ConfigurableStateTypeDefinition");
      if (!doc.setContent(&file)) {
        file.close();
        return;
      }
      file.close();

      QDomElement qde_configurableStates = doc.documentElement();
      if (qde_configurableStates.tagName() != "ConfigurableStates")
        return;

      // generate qde_configurableStateMap
      QHash<QString, QDomElement> qde_configurableStateMap;
      QDomNodeList qdn_List = qde_configurableStates.elementsByTagName("ConfigurableState");
      for (int i = 0; i < qdn_List.size(); i++)
        qde_configurableStateMap.insert(qdn_List.at(i).toElement().attribute("name"), qdn_List.at(i).toElement());
      QConfigurableLoadSaveDialog* dialog = new QConfigurableLoadSaveDialog(configurableWidgetMap, qde_configurableStateMap,
          QConfigurableLoadSaveDialog::ConfigurableLoadMultiple);
      dialog->exec();
    }

  }

  // if widget == 0, save all Configurables
  void QECBRobotsWindow::sl_saveCurrentConfigurableStatesToFile() {
    QConfigurableLoadSaveDialog* dialog = new QConfigurableLoadSaveDialog(configurableWidgetMap);
    dialog->exec();
  }

  void QECBRobotsWindow::sl_clearAutoSaveFile() {
    nodeConfigurableStateMap.clear();
    bookmarkConfigurableStates();
  }


  void QECBRobotsWindow::sl_updateLoopStateLabel() {
    QString labelString;
    switch (globalData->comm->getCurrentCommState()) {
      case QECBCommunicator::STATE_NOT_INITIALISED:
        labelString.append("NOT INITIALIZED");
        break;
      case QECBCommunicator::STATE_INITIALIZED:
        labelString.append("INITIALIZED");
        break;
      case QECBCommunicator::STATE_PAUSED:
        labelString.append("PAUSED");
        break;
      case QECBCommunicator::STATE_READY_FOR_SENDING_PACKAGE_MOTORS:
      case QECBCommunicator::STATE_READY_FOR_STEP_OVER_AGENTS:
      case QECBCommunicator::STATE_WAIT_FOR_RECEIVE_PACKAGE_SENSORS:
      case QECBCommunicator::STATE_RUNNING:
        labelString.append("RUNNING");
        break;
      case QECBCommunicator::STATE_STOPPED:
        labelString.append("STOPPED");
        break;
    }
    //QLocale().toString(globalData->controlStep, 'f', 2);
    labelString.append(" (ControlStep ").append(QLocale().toString((float)globalData->controlStep, 'f', 0)).append(")");
    loopStateLabel->setText(labelString);
  }


  void QECBRobotsWindow::enterEvent(QEvent* event) {
    event->accept();
    activateWindow();
  }

} // namespace lpzrobots
