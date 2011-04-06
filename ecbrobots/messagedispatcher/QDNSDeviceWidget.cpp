/***************************************************************************
 *   Copyright (C) 2010 by                                                 *
 *   Research Network for Self-Organization of Robot Behavior              *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    wrabe@informatik.uni-leipzig.de                                      *
 *    Georg.Martius@mis.mpg.de                                             *
 *    ralfder@mis.mpg.de                                                   *
 *    frank@nld.ds.mpg.de                                                  *
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
 *   Revision 1.4  2011-04-06 16:10:24  wrabe
 *   - swapped tooltips of status icons
 *
 *   Revision 1.3  2011/04/06 06:58:30  guettler
 *   - generated scales 20px buttons from 200px png files in order to reduce
 *     needed processing by Qt. Original 200px files are still kept in repository.
 *
 *   Revision 1.2  2011/04/05 13:06:00  guettler
 *   - typo
 *
 *   Revision 1.1  2011/04/05 12:16:04  guettler
 *   - new tabWidget
 *   - all found DNS devices are shown in tabWidget with a QDNSDeviceWidget each
 *   - QDNSDeviceWidget shows DNS device name, USB adapter name and type,
 *     response time and incoming/outgoing status (if messages are currently sent
 *     or received)
 *
 *   Revision 1.22  2011/03/22 16:38:13  guettler
 *   - adpaptions to enhanced configurable and inspectable interface:
 *   - qconfigurable is now restarted if initialization of agents is finished
 *
 *   Revision 1.21  2011/03/21 17:35:26  guettler
 *   - new autosave checkbox in context menu implemented and used
 *
 *   Revision 1.20  2011/02/11 12:12:11  guettler
 *   - UI: some seperators added
 *
 *   Revision 1.19  2011/01/28 12:15:37  guettler
 *   - restore of AutoSave File from a backup implemented
 *   - reset to original values, values AND bounds for Configurable implemented
 *   - reset to original values for tileWidgets implemented
 *
 *   Revision 1.18  2011/01/27 09:04:12  guettler
 *   - some preparations for checkbox in order to switch the autosave function
 *
 *   Revision 1.17  2011/01/24 18:40:48  guettler
 *   - autosave functionality now stores only values, bounds and descriptions of
 *   parameters if they differ from their original values
 *
 *   Revision 1.16  2011/01/05 13:28:45  guettler
 *   - bugfix: auto delete functionality of qt lead to SIGSEV by reason of wrong
 *     processing order of destructors from child (QAbstractConfigurableTileWidget) and
 *     parent (QDNSDeviceWidget) objects
 *
 *   Revision 1.15  2010/12/16 18:37:40  wrabe
 *   -added several tooltips
 *   -corrected sentences, notation, syntax for improved informational value
 *   -bugfix: if in collapsed mode, all tiles were stored as invisible
 *   -cosmetic ui changes
 *   -other minor things
 *
 *   Revision 1.14  2010/12/16 16:39:25  wrabe
 *   - drag&drop reworked: user can now drag a parameter to a any place
 *   - rearrangement of parameters now made only when user wants this
 *   - bugfixes
 *
 *   Revision 1.13  2010/12/15 18:28:34  wrabe
 *   -preparations for drag&drop of tileWidgets to empty places
 *
 *   Revision 1.12  2010/12/15 18:06:55  wrabe
 *   -regression fix: drag and drop of tileWidgets
 *
 *   Revision 1.11  2010/12/15 17:26:28  wrabe
 *   - number of colums for tileWidgets and width of tileWidgets can
 *   now be changed (independently for each Configurable)
 *   - bugfixes
 *
 *   Revision 1.10  2010/12/15 11:00:06  wrabe
 *   -load/save multiple ConfigurableStates from one file
 *   -All current ConfigurableStates can be stored and loaded now via menu
 *   -loading a ConfigurableState for one Configurable from a file containing multiple ConfigurableStates allows to choose one desired ConfigurableState
 *
 *   Revision 1.9  2010/12/14 10:10:12  guettler
 *   -autoload/autosave now uses only one xml file
 *   -fixed getName of TileWidget which produced invisible widgets in xml files
 *
 *   Revision 1.8  2010/12/13 16:22:18  wrabe
 *   - autosave function rearranged
 *   - bugfixes
 *
 *   Revision 1.7  2010/12/09 17:00:08  wrabe
 *   - load / save function of ConfigurableState (configurable + GUI)
 *   - autoload / autosave function of ConfigurableState (configurable
 *     + GUI)
 *   - handling of equal Configurable names implemented for autoload
 *     and -save
 *   - bugfixing
 *
 *   Revision 1.6  2010/12/08 17:52:57  wrabe
 *   - bugfixing/introducing new feature:
 *   - folding of the ConfigurableWidgets now awailable
 *   - highlight the ConfigurableTile when hoovered by mouse
 *   - load/store of the state of a ConfigurableWidget to file
 *
 *   Revision 1.5  2010/12/03 11:11:53  wrabe
 *   - now handled paramVal, paramInt and paramBool, all the params are displayed
 *     as ConfigurableTiles witch can be show and hide seperatly or arranged by user
 *     (showHideDialog reacheble by contextMenu (right click an the Widget containing
 *     the tiles ), arrange the Tiles is can done by drag and drop (there is no history or
 *     storage implementet yet))
 *
 *   Revision 1.4  2010/11/30 17:19:03  wrabe
 *   - bugfix
 *
 *   Revision 1.3  2010/11/30 17:07:06  wrabe
 *   - new class QConfigurableTileShowHideDialog
 *   - try to introduce user-arrangeable QConfigurationTiles (current work, not finished)
 *
 *   Revision 1.2  2010/11/28 20:33:44  wrabe
 *   - current state of work: only paramval´s
 *   - construct a configurable as a tile containing a QSlider to change the value by drag with mouse as well as a QSpinBox to change the configurable by typing new values (mouse-scrolls are also supported)
 *   - minimum and maximum boundaries can´t be changed will be so far, only a change- dialog-dummy is reacable over the context-menu
 *
 *   Revision 1.1  2010/11/26 12:22:36  guettler
 *   - Configurable interface now allows to set bounds of paramval and paramint
 *     * setting bounds for paramval and paramint is highly recommended (for QConfigurable (Qt GUI).
 *   - bugfixes
 *   - current development state of QConfigurable (Qt GUI)
 *
 *                                                                         *
 ***************************************************************************/

#include "QDNSDeviceWidget.h"
#include <QtGui>
#include "QCommunicationChannel.h"

using namespace std;

namespace lpzrobots {
  
  QDNSDeviceWidget::QDNSDeviceWidget(QCCHelper::DNSDevice_t* dnsDevice) :
    device(dnsDevice) {
    initBody();
    setAcceptDrops(true);
    setToolTip();
  }

  QDNSDeviceWidget::~QDNSDeviceWidget() {
  }

  void QDNSDeviceWidget::initBody() {
    setLayout(&layout);

    //    setTitle(QString(config->getName().c_str()) + "  -  " + QString(config->getRevision().c_str()) + "  [" + QString::number(config->getId()) + "]");
    setTitle(QString(device->dns_name));
    setFont(QFont("Courier", 11, QFont::Bold));

    QPalette pal = palette();
    pal.setColor(QPalette::AlternateBase, QColor(200, 210, 200));
    setPalette(pal);
    setBackgroundRole(QPalette::AlternateBase);
    setAutoFillBackground(true);

    // Prepare the context menu to show the configurable show and hide dialog
    setContextMenuPolicy(Qt::CustomContextMenu);
    connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(sl_execContextMenu(const QPoint &)));
//    contextMenuShowHideDialog.addAction(tr("set number of parameter columns ..."), this,
//        SLOT(sl_changeNumberTileColumns()));
//    contextMenuShowHideDialog.addSeparator();

    QInfoBox* nameWidget = new QInfoBox("USB Adapter", this);
    layout.addWidget(nameWidget, 0, 0);
    nameWidget->addLabel(device->channel->getUSBDeviceName());


    QString channelTypeString = QString();
    switch (device->channel->getUSBDeviceType()) {
      case QCCHelper::USBDevice_None:
        channelTypeString.append("no USB Device");
        break;
      case QCCHelper::USBDevice_ISP_ADAPTER:
        channelTypeString.append("ISP");
        break;
      case QCCHelper::USBDevice_USART_ADAPTER:
        channelTypeString.append("USART");
        break;
      case QCCHelper::USBDevice_XBEE_ADAPTER:
        channelTypeString.append("XBEE");
        break;
    }
    QInfoBox* typeWidget = new QInfoBox("Adapter type", this);
    layout.addWidget(typeWidget, 0, 1);//, i++, 0, Qt::AlignJustify);
    typeWidget->addLabel(channelTypeString);

    QInfoBox* responseTimeWidget = new QInfoBox("Response Time", this);
    layout.addWidget(responseTimeWidget, 0, 2);
    responseTimeLabel = responseTimeWidget->addLabel(QString::number(device->channel->getResponseTime()) + "ms");

/*    labelPixmap_USB_ISP_Adapter = new QLabel;
    labelPixmap_USB_ISP_Adapter->setBackgroundRole(QPalette::Base);
    labelPixmap_USB_ISP_Adapter->setScaledContents(false);
    labelPixmap_USB_ISP_Adapter->setPixmap(QPixmap::fromImage(QImage(":/images/USB_ISP_Adapter.png").scaledToWidth(200, Qt::SmoothTransformation)));*/

    QInfoBox* stateBox = new QInfoBox("Status", this);
    layout.addWidget(stateBox, 0, 3);
    incomingStateLabel = new QLabel();
    incomingStateLabel->setBackgroundRole(QPalette::Base);
    incomingStateLabel->setEnabled(false);
    incomingStateLabel->setScaledContents(false);
    incomingStateLabel->setEnabled(false);
    incomingStateLabel->setPixmap(QPixmap::fromImage(QImage(":/images/20px-Button_Icon_GreenForest.svg.png")));
    incomingStateLabel->setToolTip("Goes green if a package is sent to DNS device.");
    outgoingStateLabel = new QLabel();
    outgoingStateLabel->setBackgroundRole(QPalette::Base);
    outgoingStateLabel->setEnabled(false);
    outgoingStateLabel->setScaledContents(false);
    outgoingStateLabel->setPixmap(QPixmap::fromImage(QImage(":/images/20px-Button_Icon_Red.svg.png")));
    outgoingStateLabel->setToolTip("Goes red if a package is received from DNS device.");

    stateBox->addWidget(outgoingStateLabel);
    stateBox->addWidget(incomingStateLabel);

    incomingStateTimer = new QTimer(this);
    incomingStateTimer->setInterval(50);
    connect(incomingStateTimer, SIGNAL(timeout()), this, SLOT(sl_incomingStateTimeout()));
    outgoingStateTimer = new QTimer(this);
    outgoingStateTimer->setInterval(50);
    connect(outgoingStateTimer, SIGNAL(timeout()), this, SLOT(sl_outgoingStateTimeout()));

    layout.setColumnStretch(10, 100);
  }



  void QDNSDeviceWidget::sl_execContextMenu(const QPoint & pos) {
    contextMenuShowHideDialog.exec(this->mapToGlobal(pos));
  }

  void QDNSDeviceWidget::enterEvent(QEvent * event) {
    defaultPalette = palette();
    QPalette pal = QPalette(defaultPalette);
    pal.setColor(QPalette::AlternateBase, QColor(200, 200, 220));
    setPalette(pal);
    setBackgroundRole(QPalette::AlternateBase);
    setAutoFillBackground(true);
    update();
  }

  void QDNSDeviceWidget::leaveEvent(QEvent * event) {
    setPalette(defaultPalette);
    update();
  }

  void QDNSDeviceWidget::setToolTip() {
    //QGroupBox::setToolTip("Configurable is folded.\n(double click to unfold)");
  }

  void QDNSDeviceWidget::sl_dataIncoming(QCCHelper::DNSDevice_t* fromDevice) {
    if (device != fromDevice)
      return;
    responseTimeLabel->setText(QString::number(device->channel->getResponseTime()) + "ms");
    incomingStateLabel->setEnabled(true);
    incomingStateTimer->start();
  }

  void QDNSDeviceWidget::sl_dataOutgoing(QCCHelper::DNSDevice_t* toDevice) {
    if (device != toDevice)
      return;
    outgoingStateLabel->setEnabled(true);
    outgoingStateTimer->start();
  }

  void QDNSDeviceWidget::sl_outgoingStateTimeout() {
    outgoingStateTimer->stop();
    outgoingStateLabel->setEnabled(false);
  }

  void QDNSDeviceWidget::sl_incomingStateTimeout() {
    incomingStateTimer->stop();
    incomingStateLabel->setEnabled(false);
  }


} // namespace lpzrobots
