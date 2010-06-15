/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
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
 *  Defines xercesc namespace and makes some forward declarations
 *                                                                         *
 *                                                                         *
 *  $Log$
 *  Revision 1.1  2010-06-15 15:02:19  guettler
 *  using now "XercescForwardDecl.h" to avoid namespace problems (3_0, 3_1)
 *
 *                                                                         *
 **************************************************************************/
#include <xercesc/util/XercesDefs.hpp>

#define XERCESC XERCES_CPP_NAMESPACE

namespace XERCESC {
  class DOMNode;
  class DOMException;
  class SAXParseException;
}
