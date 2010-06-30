/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    mam06fyl@studserv.uni-leipzig.de (robot14)                           *
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
 *   DESCRIPTION                                                           *
 *                                                                         *
 *   Visualization tool for matrices...                                    *
 *                                                                         *
 *   $Log$
 *   Revision 1.5  2010-06-30 11:39:04  robot14
 *   removed VectorPlotChannel specs
 *
 *   Revision 1.4  2010/03/30 13:21:10  robot14
 *   added vector support
 *
 *   Revision 1.3  2009/10/14 12:22:31  robot14
 *   *** empty log message ***
 *
 *   Revision 1.2  2009/10/02 15:25:40  robot14
 *   filters, main app - not finished yet
 *
 *   Revision 1.1  2009/08/13 13:14:05  robot14
 *   first version
 *
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
	MatrixVisualizer(QWidget *parent = 0);
	virtual ~MatrixVisualizer();

	 VectorPlotChannel* getVectorPlotChannel(QString name);
	 MatrixPlotChannel* getMatrixPlotChannel(QString name);
	 void connectWindowForUpdate(VisualiserSubWidget *vis);

private:
	MatrixPipeFilter* matrix_filter;
//	VectorPipeFilter* vector_filter;
	std::list<AbstractPlotChannel*> channelList;
	std::vector<MatrixPlotChannel*> matrices;
	std::vector<VectorPlotChannel*> vectors;
	QButtonGroup *visButtons;
	// QButtonGroup::buttonClicked() emits for each button...

	QVBoxLayout* main_layout;
	QComboBox *matChoice;
	QComboBox *vizChoice;

	QHBoxLayout* makeButtons();
	configFile* config;

	void initGui();
	void linkChannels();
	static const bool debug = false;

protected:
	virtual void closeEvent(QCloseEvent * event);

private slots:
  void visualize(QAbstractButton * button);

signals:
  void sendQuit();

};

#endif /* __MATRIXVISUALIZER_H_ */
