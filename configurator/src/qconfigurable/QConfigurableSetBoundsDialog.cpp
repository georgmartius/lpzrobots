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
 *   Revision 1.1  2011/07/01 12:32:16  guettler
 *   - pull out qconfigurable part of ecb_robots to get stand the alone library libconfigurator
 *
 *   Revision 1.4  2011/03/21 17:35:44  guettler
 *   - adapted to enhanced configurable interface
 *
 *   Revision 1.3  2010/12/16 18:37:39  wrabe
 *   -added several tooltips
 *   -corrected sentences, notation, syntax for improved informational value
 *   -bugfix: if in collapsed mode, all tiles were stored as invisible
 *   -cosmetic ui changes
 *   -other minor things
 *
 *   Revision 1.2  2010/12/07 10:08:40  wrabe
 *   - disabled resizing of QConfigurableSetBoundsDialog
 *
 *   Revision 1.1  2010/12/06 17:49:34  wrabe
 *   - new QConfigurableSetBoundsDialog to change the
 *     boundaries of the Configurables (reacheble now by
 *     context menu of the ConfigurableTile (only paramval/
 *     paramint))
 *
 *                                                                         *
 ***************************************************************************/

#include "QConfigurableSetBoundsDialog.h"
#include <QLabel>
#include <QString>
#include <QDoubleValidator>
#include <QMessageBox>

namespace lpzrobots {
  
  QConfigurableSetBoundsDialog::QConfigurableSetBoundsDialog(Configurable* config, Configurable::paramkey& key, dialogMode mode) :
    config(config), key(key) {
    dialogGridLayout = new QGridLayout();
    setLayout(dialogGridLayout);
    setWindowTitle("Change the boundaries");
    setSizeGripEnabled(false);
    setMinimumWidth(335);
   // layout()->setSizeConstraint( QLayout::SetFixedSize );

     if (mode==MODE_PARAMVAL)
       createDialogContentForValBounds();
     else if (mode==MODE_PARAMINT)
       createDialogContentForIntBounds();
     else
       createDialogContentError();
   }
  
  QConfigurableSetBoundsDialog::~QConfigurableSetBoundsDialog() {
    delete (dialogGridLayout);
    switch (internal_content) {
      case content_double: {
        delete (doubleValidator);
        break;
      }
      case content_int: {
        delete (intValidator);
        break;
      }
      default:
        return;
    }
    delete (pbSetDefault);
    delete (lMinBound);
    delete (lMaxBound);
    delete (leMinBound);
    delete (leMaxBound);
    delete (buttonBox);
  }

  void QConfigurableSetBoundsDialog::createDialogContentForValBounds() {
    internal_content = content_double;
    double minBound = config->getParamvalBounds(key).first;
    double maxBound = config->getParamvalBounds(key).second;

    lMinBound = new QLabel();
    lMinBound->setText(QString(key.c_str()) + ":  minBound");

    lMaxBound = new QLabel();
    lMaxBound->setText(QString(key.c_str()) + ": maxBound");

    doubleValidator = new QDoubleValidator(this);
    doubleValidator->setNotation(QDoubleValidator::StandardNotation);

    leMinBound = new QLineEdit();
    leMinBound->setValidator(doubleValidator);
    leMinBound->setText(QString::number(minBound));

    leMaxBound = new QLineEdit();
    leMaxBound->setValidator(doubleValidator);
    leMaxBound->setText(QString::number(maxBound));

    pbSetDefault = new QPushButton(tr("defaults"));
    buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    buttonBox->addButton(pbSetDefault, QDialogButtonBox::ActionRole);

    connect(buttonBox, SIGNAL(accepted()), this, SLOT(sl_dialogAccept()));
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(sl_dialogReject()));
    connect(pbSetDefault, SIGNAL(pressed()), this, SLOT(sl_dialogSetDefaults()));

    dialogGridLayout->addWidget(lMinBound, 0, 0, 1, 1, Qt::AlignRight);
    dialogGridLayout->addWidget(lMaxBound, 1, 0, 1, 1, Qt::AlignRight);
    dialogGridLayout->addWidget(leMinBound, 0, 1, 1, 1);
    dialogGridLayout->addWidget(leMaxBound, 1, 1, 1, 1);
    dialogGridLayout->addWidget(buttonBox, 2, 0, 1, 2);
  }

  void QConfigurableSetBoundsDialog::createDialogContentForIntBounds() {
    internal_content = content_int;
    int minBound = config->getParamintBounds(key).first;
    int maxBound = config->getParamintBounds(key).second;

    lMinBound = new QLabel();
    lMinBound->setText(QString(key.c_str()) + ":  minBound");

    lMaxBound = new QLabel();
    lMaxBound->setText(QString(key.c_str()) + ": maxBound");

    intValidator = new QIntValidator(this);

    leMinBound = new QLineEdit();
    leMinBound->setValidator(intValidator);
    leMinBound->setText(QString::number(minBound));

    leMaxBound = new QLineEdit();
    leMaxBound->setValidator(intValidator);
    leMaxBound->setText(QString::number(maxBound));

    pbSetDefault = new QPushButton(tr("defaults"));
    buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    buttonBox->addButton(pbSetDefault, QDialogButtonBox::ActionRole);

    connect(buttonBox, SIGNAL(accepted()), this, SLOT(sl_dialogAccept()));
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(sl_dialogReject()));
    connect(pbSetDefault, SIGNAL(pressed()), this, SLOT(sl_dialogSetDefaults()));

    dialogGridLayout->addWidget(lMinBound, 0, 0, 1, 1, Qt::AlignRight);
    dialogGridLayout->addWidget(lMaxBound, 1, 0, 1, 1, Qt::AlignRight);
    dialogGridLayout->addWidget(leMinBound, 0, 1, 1, 1);
    dialogGridLayout->addWidget(leMaxBound, 1, 1, 1, 1);
    dialogGridLayout->addWidget(buttonBox, 2, 0, 1, 2);
  }

  void QConfigurableSetBoundsDialog::createDialogContentError() {
    internal_content = content_none;

    QString helpText;
    helpText.append("This is a dialog sets the boundaries of\n");
    helpText.append("Configurables (int/val), the dialog\n");
    helpText.append("doesn't work with a parambool!");

    QLabel lHelpText;
    lHelpText.setText(helpText);

    buttonBox->addButton(QDialogButtonBox::Close);
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(sl_dialogReject()));

    dialogGridLayout->addWidget(&lHelpText);
  }

  void QConfigurableSetBoundsDialog::sl_dialogAccept() {


    switch (internal_content) {
      case content_double: {
        bool retMinBound = false;
        bool retMaxBound = false;
        double minBound = leMinBound->text().toDouble(&retMinBound);
        double maxBound = leMaxBound->text().toDouble(&retMaxBound);

        if (retMinBound && retMaxBound) {
          config->setParamBounds(key, minBound, maxBound);
          this->accept();
        }
        break;
      }
      case content_int: {
        bool retMinBound = false;
        bool retMaxBound = false;
        int minBound = leMinBound->text().toInt(&retMinBound);
        int maxBound = leMaxBound->text().toInt(&retMaxBound);

        if (retMinBound && retMaxBound) {
          config->setParamBounds(key, minBound, maxBound);
          this->accept();
        }
        break;
      }
      default:
        this->reject();
        break;
    }
  }
  void QConfigurableSetBoundsDialog::sl_dialogReject() {
    this->reject();
  }
  void QConfigurableSetBoundsDialog::sl_dialogSetDefaults() {
    leMinBound->setText(QString::number(valDefMinBound));
    leMaxBound->setText(QString::number(valDefMaxBound));
  }
}
