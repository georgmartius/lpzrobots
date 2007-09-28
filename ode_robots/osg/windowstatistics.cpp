/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *  windowstatistics provides cool stuff for displaying statistics on the  *
 *  graphics window.                                                       *
 *                                                                         *
 *   $Log$
 *   Revision 1.3  2007-09-28 09:15:24  robot3
 *   extended comments
 *
 *   Revision 1.2  2007/09/28 08:47:29  robot3
 *   corrected some memory bug (3 still remaining)
 *
 *   Revision 1.1  2007/09/27 10:48:13  robot3
 *   first version of the WSM
 *
 *                                                                         *
 ***************************************************************************/

#include "windowstatistics.h"
#include <selforg/statisticmeasure.h>

#include "osgforwarddecl.h"
#import "color.h"
#include <osgText/Text>
#include <osgText/Font>
#include <osg/Geode>

#include <stdlib.h>


using namespace osg;

namespace lpzrobots {

WindowStatisticsManager::WindowStatisticsManager(osg::Geode* geode) : StatisticTools(), geode(geode) {
  xInitPosition = 12.0f;
  yInitPosition = 27.0f;
  zInitPosition = 0.0f;
  yOffset = 18.0f;
  font = osgText::readFontFile("fonts/arial.ttf");
  textColor = new Color(0.0,0.0,0.0,0.0);
  fontsize=12;
}

/*double& WindowStatisticsManager::addMeasure(double& observedValue, char* measureName, MeasureMode mode, long stepSpan, double additionalParam) {

  StatisticMeasure* newMeasure = new StatisticMeasure(observedValue, measureName, mode, stepSpan, additionalParam);
  this->activeMeasures.push_back(newMeasure);

  // create new text object with default settings:
  osgText::Font* font = osgText::readFontFile("fonts/arial.ttf");
  Color textColor(0.0,0.0,0.0);
  int fontsize=12;
  float textPosition = windowStatisticList.size();
  osg::Vec3 position(xInitPosition,yInitPosition+yOffset*textPosition,zInitPosition);

  osgText::Text* text = new  osgText::Text;
  geode->addDrawable( text );
  text->setCharacterSize(fontsize);
  text->setFont(font);
  text->setPosition(position);
  text->setColor(textColor);
  text->setAlignment(osgText::Text::LEFT_BASE_LINE);

  std::string buffer(newMeasure->getName());
  buffer.append(":  -");
  text->setText(buffer);

  // create WindowStatistic
  this->windowStatisticList.push_back(new WindowStatistic(newMeasure,text));

  return newMeasure->getValueAdress();
}*/

StatisticMeasure* WindowStatisticsManager::getMeasure(double& observedValue, char* measureName, MeasureMode mode, long stepSpan, double additionalParam) {

  StatisticMeasure* newMeasure = new StatisticMeasure(observedValue, measureName, mode, stepSpan, additionalParam);
  this->activeMeasures.push_back(newMeasure);

  // create new text object with default settings:
  float textPosition = windowStatisticList.size();
  osg::Vec3 position(xInitPosition,yInitPosition+yOffset*textPosition,zInitPosition);

  osgText::Text* text = new  osgText::Text;
  geode->addDrawable( text );
  text->setCharacterSize(fontsize);
  text->setFont(font);
  text->setPosition(position);
  text->setColor(*textColor);
  text->setAlignment(osgText::Text::LEFT_BASE_LINE);

  std::string buffer(newMeasure->getName());
  buffer.append(":  -");
  text->setText(buffer);

  // create WindowStatistic
  this->windowStatisticList.push_back(new WindowStatistic(newMeasure,text));

  return newMeasure;
}

void WindowStatisticsManager::doOnCallBack() {
  // first call super method (calculate new statistic values)
  StatisticTools::doOnCallBack();
  // go through WindowStatictList and update the graphical text, that should be all!
  if (StatisticTools::beginMeasureCounter==0)
    for (std::list<WindowStatistic*>::iterator i=windowStatisticList.begin();i!=windowStatisticList.end();i++) {

      char valueBuf[100];
      sprintf(valueBuf,":  %f",(*i)->getMeasure()->getValue());

      std::string buffer((*i)->getMeasure()->getName());
      buffer.append(valueBuf);

      (*i)->getText()->setText(buffer);
    }
}

}
