/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2008-08-01 14:42:04  guettler
 *   we try the trip to hell! make selforg AVR compatible...good luck (first changes)
 *
 *   Revision 1.2  2006/07/14 12:24:02  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.1  2005/11/16 11:24:27  martius
 *   moved to selforg
 *
 *   Revision 1.2  2005/11/09 13:31:51  martius
 *   GPL'ised
 *
 ***************************************************************************/
#ifndef __AVRTYPES_H
#define __AVRTYPES_H

#ifdef AVR

	// USED IN CONFIGURABLE
	const uint8_t maxNumberEntries = 5;
	const uint8_t charLength = 8;

	typedef char[charLength] charArray;

	// CONFIGURABLE
	typedef charArray paramkey;
	typedef double paramval;

	 struct parampair {
		 paramkey key;
		 paramval* val;
	  };

	 typedef parampair[maxNumberEntries] plist;

	/// INSPECTABLE
   typedef paramkey iparamkey;
   typedef paramval iparamval;

	 struct iparampair {
		 iparamkey key;
		 iparamval* val;
	  };

	 typedef iparampair[maxNumberEntries] iplist;


   typedef iparamkey[maxNumberEntries] iparamkeylist;
   typedef iparamval*[maxNumberEntries] iparamvallist;





#endif

#endif
