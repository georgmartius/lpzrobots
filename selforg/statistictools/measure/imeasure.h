/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
 ***************************************************************************/
/*
 * imeasure.h
 *
 *  Created on: 26.01.2009
 *      Author: guettler
 */

#ifndef IMEASURE_H_
#define IMEASURE_H_

/**
 * Class used by StatisticTools.
 * Provides an interface for any kind of time series analysis.
 * Every step the StatisticTools calls step.
 * @sa StatisticTools
 * @sa HUDStatisticsManager
 * @sa AbstractMeasure
 */
class IMeasure {
public:

        virtual ~IMeasure() {};

        virtual void step() = 0;

        virtual std::string getName() const = 0;

        virtual double getValue() const = 0;

        virtual double& getValueAddress()  = 0;

        virtual void setStepSize(int newStepSize) = 0;

        virtual int getStepSize() const = 0;

        virtual long getActualStep() const = 0;
};

#endif /* IMEASURE_H_ */
