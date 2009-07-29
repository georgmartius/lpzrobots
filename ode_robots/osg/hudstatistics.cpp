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
 *   Revision 1.6  2009-07-29 14:19:49  jhoffmann
 *   Various bugfixing, remove memory leaks (with valgrind->memcheck / alleyoop)
 *
 *   Revision 1.5  2008/04/29 08:45:56  guettler
 *   adapted some cosmetic changes of StatisticTools
 *
 *   Revision 1.4  2008/01/17 09:55:55  der
 *   methods added for adding std::list<AbstractMeasure*> to the HUD
 *
 *   Revision 1.3  2007/12/06 10:02:49  der
 *   abstractground: returns now cornerpoints
 *   abstractobstacle: is now trackable
 *   hudstatistics: supports now AbstractmMeasure
 *
 *   Revision 1.2  2007/09/28 12:31:49  robot3
 *   The HUDSM is not anymore deduced from StatisticalTools, so the statistics
 *   can be updated independently from the HUD
 *   addPhysicsCallbackable and addGraphicsCallbackable now exists in Simulation
 *
 *   Revision 1.1  2007/09/28 10:24:05  robot3
 *   The WindowStatisticsManager is now called HUDStatisticsManager
 *
 *   Revision 1.4  2007/09/28 10:08:49  robot3
 *   fixed memory bugs, statistics are from now on aligned right
 *
 *   Revision 1.3  2007/09/28 09:15:24  robot3
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

#include "hudstatistics.h"
#include <selforg/abstractmeasure.h>
#include <selforg/statisticmeasure.h>
#include <selforg/complexmeasure.h>
#include <selforg/statisticmeasure.h>

#include "osgforwarddecl.h"
#import "color.h"
#include <osgText/Text>
#include <osgText/Font>
#include <osg/Geode>

#include <stdlib.h>
#include <iostream>


using namespace osg;

namespace lpzrobots {

HUDStatisticsManager::HUDStatisticsManager(osg::Geode* geode, osgText::Font* font) : geode(geode), font(font)
{
  xInitPosition = 500.0f;
  yInitPosition = 27.0f;
  zInitPosition = 0.0f;
  yOffset = 18.0f;
  //font = osgText::readFontFile("fonts/arial.ttf");
  textColor = new Color(0.0,0.0,0.2,1.0);
  fontsize=12;
  statTool = new StatisticTools();
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
  text->setColor(*textColor);
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
  text->setColor(*textColor);
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



void HUDStatisticsManager::doOnCallBack() {
  // go through WindowStatictList and update the graphical text, that should be all!
  if (statTool->measureStarted())
    for (std::list<WindowStatistic*>::iterator i=windowStatisticList.begin();i!=windowStatisticList.end();i++) {

      char valueBuf[100];
      sprintf(valueBuf,":  %f",(*i)->getMeasure()->getValue());

      std::string buffer((*i)->getMeasure()->getName());
      buffer.append(valueBuf);

      (*i)->getText()->setText(buffer);
    }

}

}
