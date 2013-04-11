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
 *                                                                         *
 *                                                                         *
 *                                                                         *
 *  $Log$
 *  Revision 1.1  2009-04-17 14:17:32  guettler
 *  New PlotChannels and Filters for matrices
 *                                                                                   *
 *                                                                         *
 **************************************************************************/

#include "ExampleMatrixPipeFilter.h"
#include "MatrixPlotChannel.h"
#include "MatrixElementPlotChannel.h"

ExampleMatrixPipeFilter::ExampleMatrixPipeFilter() {
        // TODO Auto-generated constructor stub

}

ExampleMatrixPipeFilter::~ExampleMatrixPipeFilter() {
        // TODO Auto-generated destructor stub
}

virtual AbstractPlotChannel* ExampleMatrixPipeFilter::createChannel(std::string name)
{
    //if (name.find("A[0,1]")==0) return (new MotorSpeedPlotChannel("motorCspeedX"));
        if (/*ERSTER BUCHSTABE IN name großgeschrieben*/)
        { // Matrix element found!
                bool isNewChannel = false;
                // suche richtiges MatrixPlotChannel
                MatrixPlotChannel* matrixChannel;
                // wenn noch nicht in Liste:
                isNewChannel = true;
                matrixChannel = new MatrixPlotChannel(/*Großbuchstabe*/name.substr(0,1));
                this.matrixPlotChannels.push_back(matrixChannel);
                // eigentlichen Channel hinzufügen
                MatrixElementPlotChannel* elementChannel = new MatrixElementPlotChannel(/*Index*/"1");
                matrixChannel.addPlotChannel(elementChannel);
                // überlegen: evtl. Dimension der Matrix in der Hierarchie berücksichtigen (mxn)
                // addRow(GroupChannel*) verwenden usw.

                // Wenn das MatrixChannel grade erstellt, gib ihn zurück,
                // wenn bereits vorhanden (also nur Element hinzugefügt), dann gib null zurück
                if (isNewChannel)
                        return matrixChannel;
                else
                        return NULL;
        }
        else // default
                return new DefaultPlotChannel(name);
}

