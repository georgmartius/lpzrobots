/***************************************************************************
 *   Copyright (C) 2010 by                                                 *
 *   Research Network for Self-Organization of Robot Behavior              *
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
 *   Revision 1.1  2011-04-05 12:16:04  guettler
 *   - new tabWidget
 *   - all found DNS devices are shown in tabWidget with a QDNSDeviceWidget each
 *   - QDNSDeviceWidget shows DNS device name, USB adapter name and type,
 *     response time and incoming/outgoing status (if messages are currently sent
 *     or received)
 *
 *
 *                                                                         *
 ***************************************************************************/

#ifndef __QDNSDEVICEWIDGET_H_
#define __QDNSDEVICEWIDGET_H_

#include <QGroupBox>
#include <QFrame>
#include <QScrollBar>
#include <QMap>
#include <QMenu>
#include <QPalette>
#include "QCCHelper.h"

namespace lpzrobots {

  class QDNSDeviceWidget : public QGroupBox {

    Q_OBJECT

    public:
      QDNSDeviceWidget(QCCHelper::DNSDevice_t* dnsDevice);
      virtual ~QDNSDeviceWidget();

//      QCCHelper::DNSDevice_t* getDNSDevice() {
//        return device;
//      }

    public slots:
      void sl_dataIncoming(QCCHelper::DNSDevice_t* fromDevice);
      void sl_dataOutgoing(QCCHelper::DNSDevice_t* toDevice);

    protected:
      virtual void enterEvent(QEvent * event);
      virtual void leaveEvent(QEvent * event);

    private slots:
      void sl_execContextMenu(const QPoint &pos);
      void sl_outgoingStateTimeout();
      void sl_incomingStateTimeout();

    private:
      void initBody();
      void setToolTip();

      QMenu contextMenuShowHideDialog;
      QGridLayout layout;
      QPalette defaultPalette;
      QCCHelper::DNSDevice_t* device;
      QLabel* responseTimeLabel;
      QLabel* outgoingStateLabel;
      QLabel* incomingStateLabel;
      QTimer* incomingStateTimer;
      QTimer* outgoingStateTimer;

      class QInfoBox : public QGroupBox {

        public:
          QInfoBox(const QString& windowTitle, QWidget* parent) :
            QGroupBox(windowTitle, parent), _layout(new QBoxLayout(QBoxLayout::TopToBottom, this)) {
            setLayout(_layout);
            setAlignment(Qt::AlignCenter);
            setAttribute(Qt::WA_DeleteOnClose);
            setFont(QFont("Arial", 9, QFont::Bold));
          }

          virtual void addWidget(QWidget* widgetToAdd, Qt::Alignment alignment = Qt::AlignCenter) {
            _layout->addWidget(widgetToAdd, 0, alignment);
            widgetToAdd->setFont(QFont("Arial", 9, QFont::Normal));
          }

          virtual QLabel* addLabel(const QString& labelText, Qt::Alignment alignment = Qt::AlignCenter) {
            QLabel* label = new QLabel(labelText);
            addWidget(label, alignment);
            return label;
          }

        private:
          QBoxLayout* _layout;
      };

  };

}

#endif /* __QDNSDEVICEWIDGET_H_ */
