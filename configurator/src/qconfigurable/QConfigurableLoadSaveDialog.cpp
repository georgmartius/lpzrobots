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
 *   Revision 1.1  2011-07-11 16:06:01  guettler
 *   - access to Configurator is now provided by ConfiguratorProxy
 *   - creating static lib instead of dynamic variant
 *   - establish correct directory structure for including configurator into other non-qt projects
 *
 *   Revision 1.1  2011/07/01 12:32:15  guettler
 *   - pull out qconfigurable part of ecb_robots to get stand the alone library libconfigurator
 *
 *   Revision 1.4  2011/03/21 17:35:52  guettler
 *   - adapted to enhanced configurable interface
 *
 *   Revision 1.3  2011/01/24 18:40:48  guettler
 *   - autosave functionality now stores only values, bounds and descriptions of
 *   parameters if they differ from their original values
 *
 *   Revision 1.2  2010/12/16 18:37:40  wrabe
 *   -added several tooltips
 *   -corrected sentences, notation, syntax for improved informational value
 *   -bugfix: if in collapsed mode, all tiles were stored as invisible
 *   -cosmetic ui changes
 *   -other minor things
 *
 *   Revision 1.1  2010/12/15 11:00:06  wrabe
 *   -load/save multiple ConfigurableStates from one file
 *   -All current ConfigurableStates can be stored and loaded now via menu
 *   -loading a ConfigurableState for one Configurable from a file containing multiple ConfigurableStates allows to choose one desired ConfigurableState
 *
 *   Revision 1.4  2010/12/13 16:22:18  wrabe
 *   - autosave function rearranged
 *   - bugfixes
 *
 *   Revision 1.3  2010/12/06 17:49:34  wrabe
 *   - new QConfigurableSetBoundsDialog to change the
 *     boundaries of the Configurables (reacheble now by
 *     context menu of the ConfigurableTile (only paramval/
 *     paramint))
 *
 *   Revision 1.2  2010/12/03 11:11:53  wrabe
 *   - now handled paramVal, paramInt and paramBool, all the params are displayed
 *     as ConfigurableTiles witch can be show and hide seperatly or arranged by user
 *     (showHideDialog reacheble by contextMenu (right click an the Widget containing
 *     the tiles ), arrange the Tiles is can done by drag and drop (there is no history or
 *     storage implementet yet))
 *
 *   Revision 1.1  2010/11/30 17:07:06  wrabe
 *   - new class QConfigurableLoadSaveDialog
 *   - try to introduce user-arrangeable QConfigurationTiles (current work, not finished)
 *
 *                                                                         *
 ***************************************************************************/

#include "QConfigurableLoadSaveDialog.h"
#include <QDialogButtonBox>
#include <QRegion>
#include <QMessageBox>
#include <QLinkedList>
#include <QFileDialog>
#include <QCoreApplication>
#include <QTextStream>
#include <QDomNodeList>
#include <QTimer>

namespace lpzrobots {
  
  QConfigurableLoadSaveDialog::QConfigurableLoadSaveDialog(QMap<QString, QConfigurableWidget*> configurableWidgetMap) :
    configurableWidgetMap(configurableWidgetMap) {
    function = ConfigurableSave;

    setFixedHeight(200);
    setMinimumWidth(300);
    setLayout(new QVBoxLayout());
    setWindowTitle("Select the ConfigurableStates to be saved");

    cbFrame_ypos = 0;
    cbFrame = new QFrame();
    cbFrame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    grid = new QGridLayout(cbFrame);

    int row = 0;
    checkBoxConfiguableWidgetList.clear();
    foreach(QConfigurableWidget* configurableWidget, configurableWidgetMap)
      {
        QCheckBox* cb = new QCheckBox();
        cb->setText(configurableWidget->getName());
        cb->setCheckState(Qt::Checked);
        grid->addWidget(cb, row++, 0, Qt::AlignTop);
        checkBoxConfiguableWidgetList.append(cb);
      }
    scrollArea = new QScrollArea();
    scrollArea->setWidgetResizable(false);
    scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    scrollArea->setWidget(cbFrame);

    pbSelectAll = new QPushButton(tr("select all"));
    pbSelectNone = new QPushButton(tr("deselect all"));
    pbNext = new QPushButton(tr("Next >>"));
    pbCancel = new QPushButton(tr("Cancel"));

    buttonBox = new QDialogButtonBox();
    buttonBox->addButton(pbSelectAll, QDialogButtonBox::ActionRole);
    buttonBox->addButton(pbSelectNone, QDialogButtonBox::ActionRole);
    buttonBox->addButton(pbCancel, QDialogButtonBox::ActionRole);
    buttonBox->addButton(pbNext, QDialogButtonBox::ActionRole);
    connect(pbNext, SIGNAL(pressed()), this, SLOT(sl_dialogAccept()));
    connect(pbCancel, SIGNAL(pressed()), this, SLOT(reject()));
    connect(pbSelectAll, SIGNAL(pressed()), this, SLOT(sl_dialogSelectAll()));
    connect(pbSelectNone, SIGNAL(pressed()), this, SLOT(sl_dialogSelectNone()));

    layout()->addWidget(scrollArea);
    layout()->addWidget(buttonBox);
  }

  QConfigurableLoadSaveDialog::QConfigurableLoadSaveDialog(QMap<QString, QConfigurableWidget*> configurableWidgetMap,
      QHash<QString, QDomElement> qde_configurableStateMap, DialogFunction function) :
    configurableWidgetMap(configurableWidgetMap), qde_configurableStateMap(qde_configurableStateMap), function(function) {

    setFixedHeight(200);
    setMinimumWidth(440);
    setLayout(new QVBoxLayout());

    cbFrame_ypos = 0;
    cbFrame = new QFrame();
    cbFrame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    grid = new QGridLayout(cbFrame);

    int row = 0;
    checkBoxConfiguableWidgetList.clear();
    foreach(QDomElement qde_configurableState, qde_configurableStateMap)
      {
        QString name = qde_configurableState.attribute("name");
        switch (function) {
          case ConfigurableLoadSingle: {
            setWindowTitle("Select one ConfigurableState to load/use");
            QRadioButton* rb = new QRadioButton();
            rb->setText(name);
            grid->addWidget(rb, row++, 0, Qt::AlignTop);
            radioButtonConfiguableWidgetList.append(rb);
            // Default setting
            if (configurableWidgetMap.contains(name))
              rb->setChecked(true);
            break;
          }
          case ConfigurableLoadMultiple: {
            setWindowTitle("Select the ConfigurableStates to load/use");
            QCheckBox* cb = new QCheckBox();
            cb->setText(name);
            if (!configurableWidgetMap.contains(name))
              cb->setEnabled(false);
            grid->addWidget(cb, row++, 0, Qt::AlignTop);
            checkBoxConfiguableWidgetList.append(cb);
            break;
          }
          default:
            QTimer::singleShot(1, this, SLOT(reject()));
            break;
        }
      }
    scrollArea = new QScrollArea();
    scrollArea->setWidgetResizable(false);
    scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    scrollArea->setWidget(cbFrame);

    buttonBox = new QDialogButtonBox();
    if (function == ConfigurableLoadMultiple) {
      pbSelectAll = new QPushButton(tr("select all"));
      pbSelectNone = new QPushButton(tr("deselect all"));
      buttonBox->addButton(pbSelectAll, QDialogButtonBox::ActionRole);
      buttonBox->addButton(pbSelectNone, QDialogButtonBox::ActionRole);
      connect(pbSelectAll, SIGNAL(pressed()), this, SLOT(sl_dialogSelectAll()));
      connect(pbSelectNone, SIGNAL(pressed()), this, SLOT(sl_dialogSelectNone()));
    }

    pbNext = new QPushButton(tr("Ok"));
    pbCancel = new QPushButton(tr("Cancel"));
    buttonBox->addButton(pbCancel, QDialogButtonBox::ActionRole);
    buttonBox->addButton(pbNext, QDialogButtonBox::ActionRole);
    connect(pbNext, SIGNAL(pressed()), this, SLOT(sl_dialogAccept()));
    connect(pbCancel, SIGNAL(pressed()), this, SLOT(reject()));

    layout()->addWidget(scrollArea);
    layout()->addWidget(buttonBox);
  }

  QConfigurableLoadSaveDialog::~QConfigurableLoadSaveDialog() {
    delete (cbFrame);
    delete (scrollArea);
    delete (pbSelectAll);
    delete (pbSelectNone);
    delete (pbNext);
    delete (pbCancel);
  }

  void QConfigurableLoadSaveDialog::sl_dialogAccept() {
    switch (function) {
      case ConfigurableLoadSingle:
        onAcceptFunctionLoadSingle();
        break;
      case ConfigurableLoadMultiple:
        onAcceptFunctionLoadMultiple();
        break;
      case ConfigurableSave:
        onAcceptFunctionSave();
        break;
    }

    this->accept();
  }

  void QConfigurableLoadSaveDialog::onAcceptFunctionLoadMultiple() {
    foreach(QCheckBox* cbConfig, checkBoxConfiguableWidgetList)
      {
        if (cbConfig->isChecked() && cbConfig->isEnabled()) {
          configurableWidgetMap.value(cbConfig->text())->fromXml(qde_configurableStateMap.value(cbConfig->text()), false);
        }
      }
  }

  void QConfigurableLoadSaveDialog::onAcceptFunctionLoadSingle() {
    foreach(QRadioButton* rbConfig, radioButtonConfiguableWidgetList)
      {
        if (rbConfig->isChecked()) {
          configurableWidgetMap.values().at(0)->fromXml(qde_configurableStateMap.value(rbConfig->text()), false);
        }
      }
  }
  void QConfigurableLoadSaveDialog::onAcceptFunctionSave() {
    QFileDialog* fileDialog = new QFileDialog();
    fileDialog->setWindowTitle("Select the XML file to store the ConfigurableState(s)");
    fileDialog->setAcceptMode(QFileDialog::AcceptSave);
    fileDialog->setNameFilter(tr("Xml (*.xml)"));
    fileDialog->setDefaultSuffix("xml");
    QString pathApplication = QCoreApplication::applicationDirPath() + "/";
    fileDialog->selectFile(pathApplication);
    if (fileDialog->exec() == QDialog::Accepted) {
      QString fileName = fileDialog->selectedFiles().at(0);
      // construct the xml-content
      QDomDocument doc("ConfigurableStateTypeDefinition");
      // <ConfigurableStates>
      QDomElement nodeConfigurableStates = doc.createElement("ConfigurableStates");
      doc.appendChild(nodeConfigurableStates);
      foreach(QCheckBox* cbConfig, checkBoxConfiguableWidgetList)
        {
          if (cbConfig->isChecked()) {
            nodeConfigurableStates.appendChild(configurableWidgetMap.value(cbConfig->text())->toXml(true, false));
          }
        }
      QFile file(fileName);
      if (!file.open(QIODevice::WriteOnly))
        return;
      QTextStream ts(&file);
      ts << doc.toString();
      file.close();
    }
  }

  void QConfigurableLoadSaveDialog::sl_dialogSelectAll() {
    foreach(QCheckBox* cbConfigurableWidget, checkBoxConfiguableWidgetList)
      {
        if (cbConfigurableWidget->isEnabled())
          cbConfigurableWidget->setCheckState(Qt::Checked);
      }
  }

  void QConfigurableLoadSaveDialog::sl_dialogSelectNone() {
    foreach(QCheckBox* cbConfigurableWidget, checkBoxConfiguableWidgetList)
      {
        if (cbConfigurableWidget->isEnabled())
          cbConfigurableWidget->setCheckState(Qt::Unchecked);
      }
  }
}
