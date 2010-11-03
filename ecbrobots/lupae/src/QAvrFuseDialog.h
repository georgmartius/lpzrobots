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
 *   Revision 1.1  2010-11-03 13:05:27  wrabe
 *   -new version 2.0 uses ftdi driver library (libftdi and libusb)
 *   -special string identifiers (device descriptor) of usb devices (FT232RL), hard coded!
 *                                                *
 *                                                                         *
 ***************************************************************************/

#ifndef QAVRFUSEDIALOG_H_
#define QAVRFUSEDIALOG_H_

#include <QtGui>
#include <QWidget>
#include "avrDeviceList.h"
#include "types.h"

class QAvrFuseDialog : public QDialog
{
  Q_OBJECT

public:
  QAvrFuseDialog();
  virtual ~QAvrFuseDialog();

  void setAvrDevice(AVRDEVICE *d);
  AVRDEVICE* getAvrDevice();

private slots:

  void slot_OnChange_CheckState(int state);
  void LoadShippingDefaults();
  void ReadFuseBits();
  void WriteFuseBits();
  void CloseDialog();

signals:
  void readFuseBits();
  void writeFuseBits();
  void textLog(QString s);

private:

  QCheckBox    *checkBoxes[24];
  QLabel       *labelDeviceName;
  QLabel       *labelFuseBitValue[3];
  QPushButton  *pbLoadDefaultFuseBits;
  QPushButton  *pbReadFuseBits;
  QPushButton  *pbWriteFuseBits;
  QPushButton  *pbCancel;
  AVRDEVICE    *avrDevice;

};

#endif /* QAVRFUSEDIALOG_H_ */
