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
 *   Revision 1.2  2011-01-24 16:26:50  guettler
 *   -use new QLog feature
 *
 *   Revision 1.1  2010/11/23 11:08:06  guettler
 *   - some helper functions
 *   - bugfixes
 *   - better event handling
 *
 *                                                                         *
 ***************************************************************************/

#include "QMDSHelper.h"

#include <QHash>
#include <QString>

#include "QLog.h"
#include "QCommunicationChannel.h"

namespace lpzrobots {
  
  void QMDSHelper::printDNSDeviceToQCCMap(QHash<QString,QCommunicationChannel*>* map) {
     if (map->isEmpty())
       QLogWarning("No DNS devices found!");
     foreach(QString dnsDeviceName, map->keys())
       {
         // emit sig_TextLog("DNSDevice = " + dnsName + ", USBDevice = " + dnsDeviceToQCCMap[dnsDevice]->getUSBDeviceName());
         QLogVerbose("[" + dnsDeviceName + "]->[" + (*map)[dnsDeviceName]->getUSBDeviceName() + "]");
       }
   }

} // namespace lpzrobots
