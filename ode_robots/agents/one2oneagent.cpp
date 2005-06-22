/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   $Log$
 *   Revision 1.4  2005-06-22 15:34:06  fhesse
 *   fprintf in step() changed
 *
 *   Revision 1.3  2005/06/20 10:01:33  fhesse
 *   controller initialized
 *
 *   Revision 1.2  2005/06/15 14:02:47  martius
 *   revised and basicly tested
 *                                                                 *
 ***************************************************************************/
#include "one2oneagent.h"
#include "noisegenerator.h"

One2OneAgent::~One2OneAgent(){
  if(sensors) free(sensors);
  if(motors) free(motors);
  if(noise_gen) delete noise_gen;
}


bool One2OneAgent::init(AbstractController* controller, AbstractRobot* robot){
  sensornumber = robot->getSensorNumber();
  motornumber  = robot->getMotorNumber();
  controller->init(sensornumber, motornumber);

  if(!PlotAgent::init(controller, robot)) return false;  
  sensors      = (sensor*) malloc(sizeof(sensor) * sensornumber);
  motors       = (motor*)  malloc(sizeof(motor) * motornumber);
  noise_gen    = new NoiseGenerator(sensornumber);
  return true;
}


 
void One2OneAgent::step(double noise){
  if(!controller || !robot || !sensors || !motors) {
    fprintf(stderr, "%s:%i: something is null: cont %x rob %x sens %x mots %x!\n", 
	    __FILE__, __LINE__, (unsigned int)controller, (unsigned int)robot, 
	    (unsigned int)sensors, (unsigned int)motors);
  }

  int len =  robot->getSensors(sensors, sensornumber);
  if(len != sensornumber){
    fprintf(stderr, "%s:%i: Got not enough sensors!\n", __FILE__, __LINE__);
  }
  // add noise
  noise_gen->addColoredUniformlyDistributedNoise(sensors, -noise, noise);   

  controller->step(sensors, sensornumber, motors, motornumber);
  robot->setMotors(motors, motornumber);
  plot(sensors, sensornumber, motors, motornumber);
  
}
 
