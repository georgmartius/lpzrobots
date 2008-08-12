/***************************************************************************
 *   Copyright (C) 2008 by mc   *
 *   mc@linux-6hav   *
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
 ***************************************************************************/
#include "SRIRSensorWidget.h"
#include <iostream>

SRIRSensorWidget::SRIRSensorWidget ( QWidget *parent )
		: SphericalRobotSubWidget ( parent )
{

	double fa = 255./12.;
	for ( int nr=0;nr<12;nr++ )
	{
		ir_sensors[nr] = nr*fa;
		std::cout <<  ir_sensors[nr] << ", ";
	}
	std::cout << std::endl;

	ir_sensors[4] = 300;

}

void SRIRSensorWidget::setIRSensorParams ( int* ir_sensors )
{

}

void SRIRSensorWidget::paintEvent ( QPaintEvent * )
{

	QColor ir_color[12];

	QPainter painter ( this );
	painter.setRenderHint ( QPainter::Antialiasing );

//the outer shell of SphericalRobot
	painter.drawEllipse ( 10,10,this->width()-20, this->height()-20 );
	painter.drawEllipse ( 11,11,this->width()-22,this->height()-22 );
 

//Axes of SphericalRobot
	painter.drawRect ( 10,this->height() /2-2,this->width()-20,2 );
	double v = ( this->height() /2 );

	painter.setPen(Qt::gray);
	painter.drawEllipse ( 15,48,width()-30,40);
	painter.drawEllipse ( 15,112,width()-30,40);
	painter.drawRect(width()/2-1,0,2,height());
	painter.setPen(Qt::black);
//this->width()/2-(this->width()-10)/2, this->height() + (v/6+v/2)/2, this->width()-10,this->height() + (v/6+v/2)/2);

	/* order of the ir sensors
	*	                    12
	*                       10      11
	*		     7      8       9
	*		_________________________
	*		\    1      2	    3	/
	*		  \	 4	5     /
	*		     \      6     /
	*			 \     /
	*			    -
	*/


	for ( int nr=0;nr<12;nr++ )
	{
		if ( ( ir_sensors[nr]>=0 ) && ( ir_sensors[nr]<128 ) )
			ir_color[nr].setRgb ( ir_sensors[nr]*2,255,0 );
		else if ( ( ir_sensors[nr]>128 ) && ( ir_sensors[nr]<=255 ) )
			ir_color[nr].setRgb ( 255, ( 255-ir_sensors[nr] ) *2,0 );
		else ir_color[nr].setRgb ( 0,0,0 );
	}

//width and height of the painted ir-sensors
	int ir_width=10;
	int ir_height=10;


//1. IR-Sensor
	painter.setBrush ( ir_color[0] );
	painter.drawEllipse ( 20,this->height() /2+v/5+4,ir_width,ir_width );
//2. IR-Sensor
	painter.setBrush ( ir_color[1] );
	painter.drawEllipse ( this->width() /2-ir_width/2,this->height() /2+v/6,ir_width,ir_height );
//3. IR-Sensor
	painter.setBrush ( ir_color[2] );
	painter.drawEllipse ( this->width()-20-ir_width,this->height() /2+v/5+4,ir_width,ir_height );
//4. IR-Sensor
	painter.setBrush ( ir_color[3] );
	painter.drawEllipse ( this->width() /2-v/3,this->height() /2+v/2-ir_height,ir_width,ir_height );
//5. IR-Sensor
	painter.setBrush ( ir_color[4] );
	painter.drawEllipse ( this->width() /2+v/3-ir_width,this->height() /2+v/2-ir_height,ir_width,ir_height );
//6. IR-Sensor
	painter.setBrush ( ir_color[5] );
	painter.drawEllipse ( this->width() /2-ir_width/2,this->height()-15-ir_height/2-2,ir_width,ir_height );
//7. IR-Sensor
	painter.setBrush ( ir_color[6] );
	painter.drawEllipse ( 20,this->height() /2-v/5-ir_height-4,ir_width,ir_width );
//8. IR-Sensor
	painter.setBrush ( ir_color[7] );
	painter.drawEllipse ( this->width() /2-ir_width/2,this->height() /2-v/6-ir_height+2,ir_width,ir_height );
//9. IR-Sensor
	painter.setBrush ( ir_color[8] );
	painter.drawEllipse ( this->width()-20-ir_width,this->height() /2-v/5-ir_height-4,ir_width,ir_height );
//10. IR-Sensor
	painter.setBrush ( ir_color[9] );
	painter.drawEllipse ( this->width() /2-v/3,this->height() /2-v/2,ir_width,ir_height );
//11. IR-Sensor
	painter.setBrush ( ir_color[10] );
	painter.drawEllipse ( this->width() /2+v/3-ir_width,this->height() /2-v/2,ir_width,ir_height );
//12. IR-Sensor
	painter.setBrush ( ir_color[11] );
	painter.drawEllipse ( this->width() /2-ir_width/2,15+2,ir_width,ir_height );



}

