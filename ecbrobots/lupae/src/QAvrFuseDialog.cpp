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
 *   Revision 1.3  2011-01-04 13:37:50  wrabe
 *   - setting now WindowTitle at FuseBitsEditor
 *
 *   Revision 1.2  2010/11/09 17:56:55  wrabe
 *   - change of the communication protocoll between lupae and usb-isp-adapter
 *   - therefore recoding the dedicated methods
 *   - reduction of the overloded send_Message methods to one method only
 *   - insertion of QExtActions to join all events of menu-buttons as well of quickstart-buttons
 *   - adding two new functions to read out and write into the eeprom-space of the atmega128 at an ecb
 *   - change of the fontSize in the hexViewer, change of the total-width of the window
 *                                          *
 *                                                                         *
 ***************************************************************************/

#include "QAvrFuseDialog.h"
#include "mainwindow.h"


namespace lpzrobots {

  QAvrFuseDialog::QAvrFuseDialog() {
    avrDevice = NULL;

    QWidget *widget;
    QLabel *label;
    QCheckBox *checkBox;
    QGroupBox *groupBox;

    setPalette(QPalette(QColor(220, 230, 220)));
    setAutoFillBackground(true);
    setFixedSize(520, 420);

    setWindowTitle("LUPAE - Fuse-Bits-Editor");

    QGridLayout *grid = new QGridLayout();
    grid->setColumnStretch(4, 1);
    grid->setRowStretch(3, 1);
    this->setLayout(grid);

    QStringList headerNames;
    headerNames << "Fusebits Low" << "Fusebits High" << "Fusebits Ext";

    // Der Tabellenkopf
    // Die Überschriften
    // Die Zeilen der Tabelle
    {
      QFont fHugeBold("Curier", 10, QFont::Bold); //QFont serifFont("Times", 10, QFont::Bold);
      QFont fBold("Curier", 8, QFont::Bold);
      QFont fNormal("Curier", 8, QFont::Normal);

      QString sdeviceName = "unknown";
      labelDeviceName = new QLabel("AVR-Device = " + sdeviceName);
      labelDeviceName->setFont(fHugeBold);
      labelDeviceName->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
      grid->addWidget(labelDeviceName, 0, 0, 1, 3);

      label = new QLabel();
      label->setText("Note that the fuses are read as logical zero, '0', if they are programmed.");
      label->setMinimumHeight(40);
      label->setFont(fBold);
      label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
      grid->addWidget(label, 1, 0, 1, 3);

      for (int i = 0; i < 3; i++)
      {
        groupBox = new QGroupBox(headerNames.at(i));
        groupBox->setFont(fHugeBold);
        groupBox->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        grid->addWidget(groupBox, 2, i);

        QGridLayout *gridGroupBox = new QGridLayout();
        gridGroupBox->setColumnMinimumWidth(0, 10);
        groupBox->setLayout(gridGroupBox);

        label = new QLabel();
        label->setText("Bit");
        label->setFont(fBold);
        label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        gridGroupBox->addWidget(label, 0, 1, Qt::AlignHCenter);

        widget = new QWidget();
        widget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        widget->setMinimumWidth(10);
        widget->setMaximumWidth(20);
        gridGroupBox->addWidget(widget, 0, 2);

        label = new QLabel();
        label->setText("Name");
        label->setFont(fBold);
        label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        gridGroupBox->addWidget(label, 0, 3, 1, 2);

        for (int j = 0; j < 8; j++)
        {
          label = new QLabel();
          label->setText(QString::number(7 - j));
          label->setFont(fNormal);
          label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
          gridGroupBox->addWidget(label, 1 + j, 1, Qt::AlignHCenter);

          checkBox = new QCheckBox();
          checkBox->setText("-");
          checkBox->setFont(fNormal);
          checkBox->setChecked(false);
          checkBox->setEnabled(false);
          checkBox->setMinimumWidth(110);
          checkBox->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
          gridGroupBox->addWidget(checkBox, 1 + j, 2, 1, 2);

          connect(checkBox, SIGNAL(stateChanged(int)), this, SLOT(slot_OnChange_CheckState(int)));

          checkBoxes[i * 8 + (7 - j)] = checkBox;
        } // end for(j)

        label = new QLabel();
        label->setText("Value = 0x" + QString::number(0, 16));
        label->setFont(fBold);
        label->setMinimumHeight(30);
        label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        gridGroupBox->addWidget(label, 9, 1, 1, 3);
        labelFuseBitValue[i] = label;

      } // end for(i)

      QWidget *widgetButtons = new QWidget();
      QGridLayout *gridButtons = new QGridLayout();
      gridButtons->setColumnStretch(0, 1);
      widgetButtons->setLayout(gridButtons);
      grid->addWidget(widgetButtons, 4, 0, 1, 3);

      // some PushButtons
      pbCancel = new QPushButton();
      pbCancel->setText("Cancel");
      pbCancel->setFont(fBold);
      pbCancel->setMinimumHeight(30);
      pbCancel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
      gridButtons->addWidget(pbCancel, 0, 4);

      pbLoadDefaultFuseBits = new QPushButton();
      pbLoadDefaultFuseBits->setText("Load shipping defaults");
      pbLoadDefaultFuseBits->setFont(fBold);
      pbLoadDefaultFuseBits->setMinimumHeight(30);
      pbLoadDefaultFuseBits->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
      gridButtons->addWidget(pbLoadDefaultFuseBits, 0, 1);

      pbReadFuseBits = new QPushButton();
      pbReadFuseBits->setText("Read");
      pbReadFuseBits->setFont(fBold);
      pbReadFuseBits->setMinimumHeight(30);
      pbReadFuseBits->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
      gridButtons->addWidget(pbReadFuseBits, 0, 2);

      pbWriteFuseBits = new QPushButton();
      pbWriteFuseBits->setText("Write");
      pbWriteFuseBits->setFont(fBold);
      pbWriteFuseBits->setMinimumHeight(30);
      pbWriteFuseBits->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
      pbWriteFuseBits->setEnabled(false);
      gridButtons->addWidget(pbWriteFuseBits, 0, 3);

      connect(pbCancel, SIGNAL(clicked()), this, SLOT(CloseDialog()));
      connect(pbLoadDefaultFuseBits, SIGNAL(clicked()), this, SLOT(LoadShippingDefaults()));
      connect(pbReadFuseBits, SIGNAL(clicked()), this, SLOT(ReadFuseBits()));
      connect(pbWriteFuseBits, SIGNAL(clicked()), this, SLOT(WriteFuseBits()));
    }
    //*/
  }

  QAvrFuseDialog::~QAvrFuseDialog() {
  }

  AVRDEVICE* QAvrFuseDialog::getAvrDevice() {
    return avrDevice;
  }

  void QAvrFuseDialog::setAvrDevice(AVRDEVICE *d) {
    QCheckBox *checkBox;
    QString *name;
    bool checked, enabled;

    avrDevice = d;

    if (avrDevice != 0)
    {
      // Setze den Namen des TargetDevice
      labelDeviceName->setText("AVR-Device = " + *avrDevice->Name);

      uint fuseValues[3];
      fuseValues[0] = ((uint) avrDevice->FuseBytes[0]->Value) & 0xFF;
      fuseValues[1] = ((uint) avrDevice->FuseBytes[1]->Value) & 0xFF;
      fuseValues[2] = ((uint) avrDevice->FuseBytes[2]->Value) & 0xFF;

      for (int i = 0; i < 3; i++)
      {
        // Stelle den Wert des FuseBytes dar.
        labelFuseBitValue[i]->setText("Value = " + QString::number(fuseValues[i]) + " (0x" + QString::number(fuseValues[i], 16).toUpper() + ")");

        for (int j = 0; j < 8; j++)
        {
          checkBox = checkBoxes[i * 8 + (7 - j)];
          uint fuseBit = fuseValues[i] & (1 << (7 - j));
          uint enableMask = avrDevice->FuseBytes[i]->EnabledBitMask & (1 << (7 - j));

          // Text
          name = avrDevice->FuseBytes[i]->Names[7 - j];
          // CheckState
          checked = ((fuseBit > 0) ? true : false);
          // EnableState
          enabled = ((enableMask > 0) ? true : false);

          checkBox->setText((name != 0) ? *name : "-");
          checkBox->setChecked(checked);
          checkBox->setEnabled(enabled);
        }//for j
      }// for i
    }// if
    else
    {
      // Setze den Namen des TargetDevice
      // labelDeviceName->setText("Unbekanntes Zeilsystem.");

      for (int i = 0; i < 3; i++)
      {
        // Stelle den Wert des FuseBytes dar.
        labelFuseBitValue[i]->setText("Value = 0");

        for (int j = 0; j < 8; j++)
        {
          checkBox = checkBoxes[i * 8 + (7 - j)];
          checkBox->setText("-");
          checkBox->setChecked(false);
          checkBox->setEnabled(false);
        }//for j
      }// for i
    }
  }
  //void QAvrFuseDialog::setTargetDeviceName(QString name)
  //{
  //  labelDeviceName->setText(name);
  //}


  void QAvrFuseDialog::slot_OnChange_CheckState(int state) {
    QCheckBox *checkBox;
    QByte value;

    if (avrDevice != 0)
    {
      for (int i = 0; i < 3; i++)
      {
        value = 0;

        for (int j = 0; j < 8; j++)
        {
          checkBox = checkBoxes[i * 8 + (7 - j)];

          if (checkBox->isChecked())
            value += 1 << (7 - j);
        }//for j

        // Stelle den Wert des FuseBytes dar.
        labelFuseBitValue[i]->setText("Value = " + QString::number(value) + " (0x" + QString::number(value, 16).toUpper() + ")");

        avrDevice->FuseBytes[i]->Value = value;
      }// for i
    }// if
  }

  void QAvrFuseDialog::LoadShippingDefaults() {
    avrDevice->FuseBytes[0]->Value = avrDevice->FuseBytes[0]->DefaultValue;
    avrDevice->FuseBytes[1]->Value = avrDevice->FuseBytes[1]->DefaultValue;
    avrDevice->FuseBytes[2]->Value = avrDevice->FuseBytes[2]->DefaultValue;

    setAvrDevice(avrDevice);
    pbWriteFuseBits->setEnabled(true);
    emit textLog("FuseBits Defaults loaded");
  }
  void QAvrFuseDialog::ReadFuseBits() {
    pbWriteFuseBits->setEnabled(true);
    emit readFuseBits(MainWindow::EVENT_ISP_AVRDEVICE_FUSES_READ);
  }
  void QAvrFuseDialog::WriteFuseBits() {
    emit writeFuseBits(MainWindow::EVENT_ISP_AVRDEVICE_FUSES_WRITE);
  }

  void QAvrFuseDialog::CloseDialog() {
    done(0);
  }

}//namespace lpzrobots
