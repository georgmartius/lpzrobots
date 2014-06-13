/***************************************************************************
 *   Copyright (C) 2008-2011 LpzRobots development team                    *
 *    Antonia Siegert (original author)                                  *
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

#ifndef __MATRIXVISUALIZER_H_
#define __MATRIXVISUALIZER_H_

#include "AbstractRobotGUI.h"

#include "MatrixPipeFilter.h"
//#include "VectorPipeFilter.h"
#include "SimplePipeReader.h"

#include "VisualiserSubWidget.h"
#include "MatrixElementPlotChannel.h"
#include "MatrixPlotChannel.h"
#include "VectorElementPlotChannel.h"
#include "VectorPlotChannel.h"
#include "configFile.h"
#include <list>
#include <vector>

class configFile;

class MatrixVisualizer: public AbstractRobotGUI
{
  Q_OBJECT


public:
        MatrixVisualizer(QWidget *parent = 0, bool novideo = false);
        virtual ~MatrixVisualizer();

         VectorPlotChannel* getVectorPlotChannel(QString name);
         MatrixPlotChannel* getMatrixPlotChannel(QString name);
         void connectWindowForUpdate(VisualiserSubWidget *vis);

private:
        MatrixPipeFilter* matrix_filter;
//        VectorPipeFilter* vector_filter;
        std::list<AbstractPlotChannel*> channelList;
        std::vector<MatrixPlotChannel*> matrices;
        std::vector<VectorPlotChannel*> vectors;
        QButtonGroup *visButtons;
        // QButtonGroup::buttonClicked() emits for each button...

        QVBoxLayout* main_layout;
        QComboBox *matChoice;
        QComboBox *vizChoice;
        QLabel *nameLabel;

        QHBoxLayout* makeButtons();
        configFile* config;

        QString srcName;

        void initGui();
        void linkChannels();
        static const bool debug = false;

protected:
        virtual void closeEvent(QCloseEvent * event);

private slots:
  void visualize(QAbstractButton * button);
  void sourceName(QString name);
  void captureFrame(long idx, QString directory);

signals:
  void sendQuit();

};

#endif /* __MATRIXVISUALIZER_H_ */
