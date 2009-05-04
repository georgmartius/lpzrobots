/***************************************************************************
 *   Copyright (C) 2005-2009 by Robot Group Leipzig                        *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    jhoffmann@informatik.uni-leipzig.de                                  *
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
 *   Informative Beschreibung der Klasse                                   *
 *                                                                         *
 *   $Log$
 *   Revision 1.4  2009-05-04 07:06:14  robot12
 *   some implements... Part6
 *
 *   Revision 1.3  2009/04/30 14:32:34  robot12
 *   some implements... Part5
 *
 *   Revision 1.2  2009/04/30 11:35:53  robot12
 *   some changes:
 *    - insert a SelectStrategie
 *    - insert a MutationStrategie
 *    - reorganisation of the design
 *
 *
 *
 ***************************************************************************/

#ifndef FIXGENERATIONSIZESTRATEGIE_H_
#define FIXGENERATIONSIZESTRATEGIE_H_

#include "types.h"

#include "IGenerationSizeStrategie.h"
#include "Generation.h"

class FixGenerationSizeStrategie: public IGenerationSizeStrategie {
public:
	FixGenerationSizeStrategie(int value);
	virtual ~FixGenerationSizeStrategie();

	virtual int calcGenerationSize(Generation* oldGeneration);

protected:
	int m_size;

private:
	FixGenerationSizeStrategie();
};

#endif /* FIXGENERATIONSIZESTRATEGIE_H_ */
