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
 *   Revision 1.3  2009-04-29 14:32:29  robot12
 *   some implements... Part4
 *
 *   Revision 1.2  2009/04/29 11:36:41  robot12
 *   some implements... Part3
 *
 *   Revision 1.1  2009/04/27 10:59:34  robot12
 *   some implements
 *
 *
 ***************************************************************************/

#ifndef SINGLETONGENALGAPI_H_
#define SINGLETONGENALGAPI_H_

#include "types.h"

#include <string>

#include "IGenerationSizeStrategie.h"
#include "IFitnessStrategie.h"
#include "SingletonGenEngine.h"
#include "FixMutationFactorStrategie.h"
#include "StandartMutationFactorStrategie.h"
#include "FixGenerationSizeStrategie.h"
#include "StandartGenerationSizeStrategie.h"
#include "GenPrototyp.h"
#include "IRandomStrategie.h"
#include "IMutationFactorStrategie.h"
#include "GenContext.h"
#include "Gen.h"
#include "Individual.h"
#include "IValue.h"
#include "TemplateValue.h"

class SingletonGenAlgAPI {
public:
	inline void select(bool createNextGeneration=true) {SingletonGenEngine::getInstance()->select(createNextGeneration);}
	inline void crosover(RandGen* random) {if(random!=NULL)SingletonGenEngine::getInstance()->crosover(random);}

	inline void setGenerationSizeStrategie(IGenerationSizeStrategie* strategie) {Generation::setGenerationSizeStrategie(strategie);}
	inline void setFitnessStrategie(IFitnessStrategie* strategie) {Individual::setFitnessStrategie(strategie);}

	inline SingletonGenEngine* getEngine(void)const {return SingletonGenEngine::getInstance();}

	inline IMutationFactorStrategie* createFixMutationFactorStrategie(IValue* value)const {return new FixMutationFactorStrategie(value);}
	inline IMutationFactorStrategie* createStandartMutationFactorStrategie(void)const {return new StandartMutationFactorStrategie();}
	inline IGenerationSizeStrategie* createFixGenerationSizeStrategie(void)const {return new FixGenerationSizeStrategie();}
	inline IGenerationSizeStrategie* createStandartGenerationSizeStrategie(void)const {return new StandartGenerationSizeStrategie();}

	inline GenPrototyp* createPrototyp(std::string name, IRandomStrategie* strategie)const {return new GenPrototyp(name,strategie);}
	inline GenContext* createContext(GenPrototyp prototyp, IMutationFactorStrategie* strategie)const {return new GenContext(strategie,prototyp);}

	inline IValue* createDoubleValue(double value)const {return new TemplateValue<double>(value);}

	inline static SingletonGenAlgAPI* getInstance(void) {if(m_api==0)m_api = new SingletonGenAlgAPI();return m_api;}
	inline static void detroyAPI(void) {delete m_api;}

protected:
	static SingletonGenAlgAPI* m_api = 0;

private:
	SingletonGenAlgAPI();
	virtual ~SingletonGenAlgAPI();
};

#endif /* SINGLETONGENALGAPI_H_ */
