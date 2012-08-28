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

#include "hudstatistics.h"
#include <selforg/abstractmeasure.h>
#include <selforg/statisticmeasure.h>
#include <selforg/complexmeasure.h>
#include <selforg/statisticmeasure.h>

#include "osgforwarddecl.h"
#include "color.h"
#include <osgText/Text>
#include <osgText/Font>
#include <osg/Geode>

#include <stdlib.h>
#include <iostream>
#include <cstdio>


using namespace osg;

namespace lpzrobots {

  HUDStatisticsManager::HUDStatisticsManager(osg::Geode* geode, osgText::Font* font, int ypos) : geode(geode), font(font), textColor(0.0,0.0,0.2,1.0)
  {
    xInitPosition = 500.0f;
    yInitPosition = ypos;
    zInitPosition = 0.0f;
    fontsize=12;
    yOffset = fontsize + 4;
    statTool = new StatisticTools();
  }

  HUDStatisticsManager::~HUDStatisticsManager() {
    delete(statTool);
  }


StatisticMeasure* HUDStatisticsManager::getMeasure(double& observedValue, const char* measureName, MeasureMode mode, long stepSpan, double additionalParam) {

  StatisticMeasure* newMeasure = this->statTool->getMeasure(observedValue, measureName, mode, stepSpan, additionalParam);

  // create new text object with default settings:
  float textPosition = windowStatisticList.size();
  osg::Vec3 position(xInitPosition,yInitPosition+yOffset*textPosition,zInitPosition);

  osgText::Text* text = new  osgText::Text;
  geode->addDrawable( text );
  text->setCharacterSize(fontsize);
  text->setFont(font);
  text->setPosition(position);
  text->setColor(textColor);
  text->setAlignment(osgText::Text::RIGHT_BASE_LINE);

  std::string buffer(newMeasure->getName());
  buffer.append(":  -");
  text->setText(buffer);

  // create WindowStatistic
  this->windowStatisticList.push_back(new WindowStatistic(newMeasure,text));
  return newMeasure;
}

double& HUDStatisticsManager::addMeasure(double& observedValue, const char* measureName, MeasureMode mode, long stepSpan, double additionalParam) {

  StatisticMeasure* newMeasure = this->getMeasure(observedValue, measureName, mode, stepSpan, additionalParam);

  return newMeasure->getValueAddress();
}

double& HUDStatisticsManager::addMeasure(AbstractMeasure* measure) {
   // create new text object with default settings:
  float textPosition = windowStatisticList.size();
  osg::Vec3 position(xInitPosition,yInitPosition+yOffset*textPosition,zInitPosition);

  osgText::Text* text = new  osgText::Text;
  geode->addDrawable( text );
  text->setCharacterSize(fontsize);
  text->setFont(font);
  text->setPosition(position);
  text->setColor(textColor);
  text->setAlignment(osgText::Text::RIGHT_BASE_LINE);

  std::string buffer(measure->getName());
  buffer.append(":  -");
  text->setText(buffer);

  // create WindowStatistic
  this->windowStatisticList.push_back(new WindowStatistic(measure,text));
  this->statTool->addMeasure(measure);
  return  measure->getValueAddress();
}

double& HUDStatisticsManager::addMeasureList(std::list<AbstractMeasure*> measureList) {
  FOREACH(std::list<AbstractMeasure*>,measureList,measure) {
    addMeasure(*measure);
  }
  return measureList.front()->getValueAddress();
}

double& HUDStatisticsManager::addMeasureList(std::list<ComplexMeasure*> measureList) {
  FOREACH(std::list<ComplexMeasure*>,measureList,measure) {
    addMeasure(*measure);
  }
  return measureList.front()->getValueAddress();
}

double& HUDStatisticsManager::addMeasureList(std::list<StatisticMeasure*> measureList) {
  FOREACH(std::list<StatisticMeasure*>,measureList,measure) {
    addMeasure(*measure);
  }
  return measureList.front()->getValueAddress();
}

HUDStatisticsManager::WindowStatistic* HUDStatisticsManager::getMeasureWS(const std::string& measureName){
  FOREACH(std::list<WindowStatistic*>, windowStatisticList, i) {
    if((*i)->getMeasure()->getName() == measureName){
      return *i;
    }
  }
  return 0;
}


void HUDStatisticsManager::doOnCallBack(BackCaller* source, BackCaller::CallbackableType /* = BackCaller::DEFAULT_CALLBACKABLE_TYPE */) {
  // go through WindowStatictList and update the graphical text, that should be all!
  if (statTool->measureStarted())
    FOREACHC(std::list<WindowStatistic*>, windowStatisticList, i) {
      char valueBuf[100];
      char printstr[24];
      sprintf(printstr, ": %%.%if", (*i)->getMeasure()->getDisplayPrecision());

      sprintf(valueBuf,printstr,(*i)->getMeasure()->getValue());

      std::string buffer((*i)->getMeasure()->getName());
      buffer.append(valueBuf);

      (*i)->getText()->setText(buffer);
    }

}

}
