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
 *
 *                                                                         *
 *                                                                         *
 *  $Log$
 *  Revision 1.1  2010-03-07 22:50:38  guettler
 *  first development state for feature XMLImport
 *                       *
 *                                                                         *
 **************************************************************************/
#include "XMLErrorHelper.h"

#include <xercesc/util/XMLString.hpp>

#include <xercesc/sax/SAXParseException.hpp>
#include <xercesc/dom/DOMException.hpp>

#include <iostream>
#include <stdlib.h>
#include <string.h>

#include "XMLHelper.h"

using namespace std;
using namespace xercesc;

XMLErrorHelper::XMLErrorHelper()  : errorsSeen(false) { }

XMLErrorHelper::~XMLErrorHelper() {}

void XMLErrorHelper::warning(const SAXParseException& exception) {
  cerr << "Warning";
  XMLErrorHelper::printError(exception);
}

void XMLErrorHelper::error(const SAXParseException& exception) {
  errorsSeen = true;
  cerr << "Error";
  XMLErrorHelper::printError(exception);
}

void XMLErrorHelper::fatalError(const SAXParseException& exception) {
  errorsSeen = true;
  cerr << "Error";
  XMLErrorHelper::printError(exception);
}

void XMLErrorHelper::printError(const SAXParseException& exception) {
  cerr << " at file \"" << C(exception.getSystemId()) << "\", line " << exception.getLineNumber();
  cerr << ", column " << exception.getColumnNumber() << ":" << endl << "Message: " << C(exception.getMessage()) << endl;
}

void XMLErrorHelper::printError(const DOMException& exception) {
  cerr << "Error (DOMException): Code \"" << exception.code << "\"" << endl;
  cerr << "Message: " << C(exception.getMessage()) << endl;
}

void XMLErrorHelper::printError(const string message) {
  cerr << "Error: " << message << endl;
}

void XMLErrorHelper::resetErrors() {
  errorsSeen = false;
}

bool XMLErrorHelper::getSawErrors() const {
  return errorsSeen;
}


