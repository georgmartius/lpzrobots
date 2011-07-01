/***************************************************************************
 *   Copyright (C) 2011 by Robot Group Leipzig                             *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    wrabe@informatik.uni-leipzig.de                                      *
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
 *   Revision 1.1  2011-07-01 12:32:15  guettler
 *   - pull out qconfigurable part of ecb_robots to get stand the alone library libconfigurator
 *
 *   Revision 1.32  2011/04/06 06:48:59  guettler
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

#include "QConfigurator.h"
#include "QConfigurableLoadSaveDialog.h"
#include  <QScrollArea>

namespace lpzrobots {

  QConfigurator::QConfigurator(QString applicationPath, ConfigList& configList) :
    applicationPath(applicationPath), configList(configList), configWidget(0), isClosed(false) {

    this->setWindowTitle("LpzRobots Configurator V1.0");
    statusLabel = new QLabel(statusBar());

    statusBar()->addWidget(statusLabel, 1);
    connect(&statusLabelTimer, SIGNAL(timeout()), this, SLOT(sl_statusLabelTimerExpired()));

    autoloadConfigurableStates();

    logView = new QLogViewWidget();

    // Layout:
    tabWidget = new QTabWidget;
    tabWidget->addTab(logView, tr("Report"));
    setCentralWidget(tabWidget);

    createActions();
    createMenus();

    updateConfigurableWidget();

    readSettings();
  }

  void QConfigurator::createActions() {
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

    sl_GUIEventHandler(EVENT_SWITCH_WARNING);
    sl_GUIEventHandler(EVENT_SWITCH_VERBOSE);
    sl_GUIEventHandler(EVENT_SWITCH_DEBUG);

    // Actions About
    action_About = new QAction(tr("About ..."), this);
    action_About->setStatusTip(tr("Shows the application's About box."));
    connect(action_About, SIGNAL(triggered()), this, SLOT(sl_About()));

  }

  void QConfigurator::createMenus() {
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

//    additionalsMenu = menuBar()->addMenu(tr("&Additionals"));

    settingsMenu = menuBar()->addMenu(tr("&Settings"));
    settingsMenu->addAction(action_SwitchWarning);
    settingsMenu->addAction(action_SwitchVerbose);
    settingsMenu->addAction(action_SwitchDebug);

    helpMenu = menuBar()->addMenu(tr("&Help"));
    helpMenu->addAction(action_About);
  }

  void QConfigurator::readSettings() {
    QSettings settings(applicationPath + QString("configurator.settings"), QSettings::IniFormat);
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
    warningOutputEnabled = warning;
    verboseOutputEnabled = verbose;
    debugOutputEnabled = debug;
    tabWidget->setCurrentIndex(settings.value("activeTabIndex", 1).toInt());
  }

  void QConfigurator::writeSettings() {
    QSettings settings(applicationPath + QString("configurator.settings"), QSettings::IniFormat);
    settings.setValue("pos", pos());
    settings.setValue("size", size());
    settings.setValue("warningOutput", warningOutputEnabled);
    settings.setValue("verboseOutput", verboseOutputEnabled);
    settings.setValue("debugOutput", debugOutputEnabled);
    settings.setValue("activeTabIndex", tabWidget->currentIndex());
  }

  void QConfigurator::closeEvent(QCloseEvent *event) {
    // Vermutung: es bleibt ein Object erhalten, sodass dadurch dieses
    // Fenster nicht beendet.
    // Folge: fehler in der Autosave-Funktion
    // TODO: behebe es ...

    if (!isClosed) {
      //      QString text = "QConfigurator::closeEvent(";
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

  void QConfigurator::sl_textLog(QString sText) {
    statusLabelTimer.stop();
    statusLabel->setText(sText);
    statusLabelTimer.start(5000);
    logView->appendLogViewText(sText);
  }

  void QConfigurator::sl_statusLabelTimerExpired() {
    statusLabel->setText("");
  }

  void QConfigurator::sl_ClearLogView() {
    logView->clearLogViewText();
  }

  void QConfigurator::sl_Close() {
    close();
  }

  void QConfigurator::sl_About() {
    QMessageBox::about(this, tr("About the Application"), tr(
        "LpzRobots Configurator V1.0, a tool to configure e.g. real and simulated robots or their controllers live."));
  }



  void QConfigurator::sl_GUIEventHandler(int eventCode) {
    switch (eventCode) {
      case EVENT_SWITCH_WARNING:
        warningOutputEnabled = action_SwitchWarning->isChecked();
        sl_textLog("Set warning output to " + QString::number(warningOutputEnabled));
        break;
      case EVENT_SWITCH_VERBOSE:
        verboseOutputEnabled = action_SwitchVerbose->isChecked();
        sl_textLog("Set verbose output to " + QString::number(verboseOutputEnabled));
        break;
      case EVENT_SWITCH_DEBUG:
        debugOutputEnabled = action_SwitchDebug->isChecked();
        sl_textLog("Set debug output to " + QString::number(debugOutputEnabled));
        break;
      default:
        break;
    }
  }

  void QConfigurator::restoreOriginalConfigurables() {
    // manual delete so they can restore the original parameters of the Configurables
    foreach(QConfigurableWidget* configWidget, configurableWidgetMap) {
      configWidget->sl_resetToOriginalValuesAndBounds();
    }
  }

  void QConfigurator::updateConfigurableWidget() {
    configurableWidgetMap.clear();
    int index = tabWidget->currentIndex();
    tabWidget->removeTab(1);
    tabWidget->insertTab(1, createConfigurableWidget(), "Configurables");
    tabWidget->setCurrentIndex(index);
  }

  int QConfigurator::addConfigurablesToGrid(const ConfigList& configList, QGridLayout* grid, QHash<QString, int>& configurableIndexMap, int configurableWidgetIndex /*= 0*/, int embeddingDepth) {
    FOREACHC(ConfigList, configList, config) {
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

  QWidget* QConfigurator::createConfigurableWidget() {
    if (configWidget != 0)
      delete configWidget;
    configWidget = new QWidget(); // containing some QGroupBoxes (QConfigurableWidget´s)
    QGridLayout* grid = new QGridLayout();
    grid->setSizeConstraint(QLayout::SetFixedSize);
    configWidget->setLayout(grid);
    QHash<QString, int> configurableIndexMap;
    int numberWidgetsAdded = addConfigurablesToGrid(configList, grid, configurableIndexMap);
    recallConfigurableStates(); // autoload function
    grid->setRowStretch(numberWidgetsAdded, 100);
    scrollArea = new QScrollArea();
    //scrollArea->setBackgroundRole(QPalette::Dark);
    scrollArea->setWidget(configWidget);
    scrollArea->setToolTip(QString::number(numberWidgetsAdded) + " Configurables");
    return scrollArea;
  }

  // is called when a configurable or one of their childs has been changed
  void QConfigurator::sl_configurableChanged(QConfigurableWidget* sourceWidget) {
    // simplest way: just recreate entire scrollarea
    bookmarkConfigurableStates();
    restoreOriginalConfigurables();
    updateConfigurableWidget();
  }

  void QConfigurator::autostoreConfigurableStates() {
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

  void QConfigurator::autoloadConfigurableStates() {
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

  void QConfigurator::sl_restoreAutoSaveFile() {
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

  void QConfigurator::recallConfigurableStates() {
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

  void QConfigurator::bookmarkConfigurableStates() {
    // if some older ones exist, just overwrite them
    foreach(QConfigurableWidget* confWidget, configurableWidgetMap)
        nodeConfigurableStateMap.insert(confWidget->getName(), confWidget->toXml(false, true));
  }

  void QConfigurator::sl_loadCurrentConfigurableStatesFromFile() {
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
  void QConfigurator::sl_saveCurrentConfigurableStatesToFile() {
    QConfigurableLoadSaveDialog* dialog = new QConfigurableLoadSaveDialog(configurableWidgetMap);
    dialog->exec();
  }

  void QConfigurator::sl_clearAutoSaveFile() {
    nodeConfigurableStateMap.clear();
    bookmarkConfigurableStates();
  }


   void QConfigurator::enterEvent(QEvent* event) {
    event->accept();
    activateWindow();
  }

} // namespace lpzrobots
