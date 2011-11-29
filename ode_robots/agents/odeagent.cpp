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

#include "odeagent.h"
#include "oderobot.h"
#include "globaldata.h"
#include "joint.h"
#include "pos.h"
#include "tmpprimitive.h"
#include <assert.h>

namespace lpzrobots {


  OdeAgent::OdeAgent(const PlotOption& plotOption, double noisefactor, const std::string& name, const std::string& revision)
    : Agent(plotOption, noisefactor, name, revision) {
    constructor_helper(0);
  }
  OdeAgent::OdeAgent(const std::list<PlotOption>& plotOptions, double noisefactor, const std::string& name, const std::string& revision)
    : Agent(plotOptions, noisefactor, name, revision) {
    constructor_helper(0);
  }
    
  OdeAgent::OdeAgent(const GlobalData& globalData, double noisefactor, const std::string& name, const std::string& revision)
    : Agent(globalData.plotoptions, noisefactor, name, revision){
    constructor_helper(&globalData);
  }

  OdeAgent::OdeAgent(const GlobalData& globalData, const PlotOption& plotOption, 
                     double noisefactor, const std::string& name, const std::string& revision)
    : Agent(plotOption, noisefactor, name, revision){
    constructor_helper(&globalData);
  }


  OdeAgent::OdeAgent(const GlobalData& globalData, const PlotOptionList& plotOptions, 
                     double noisefactor, const std::string& name, const std::string& revision)
    : Agent(plotOptions, noisefactor, name, revision){
    constructor_helper(&globalData);
  }
  
  OdeAgent::~OdeAgent(){
    removeOperators();
  }
  
  void OdeAgent::constructor_helper(const GlobalData* globalData){
    fixateRobot         = false;
    fixedJoint          = 0;
    tracing_initialized = false;
    trace_length        = 1000;
    trace_thickness     = 0.05;
    if(globalData){
      FOREACHC(std::list<Configurable*>, globalData->globalconfigurables, c){
        plotEngine.addConfigurable(*c);
      }
    }
  }


  bool OdeAgent::setTraceLength(int tracelength){
    if(tracing_initialized==true || tracelength < 1){
      return false;
    }else{
      this->trace_length = tracelength;
      return true;
    }
  }

  void OdeAgent::init_tracing(){

    segments = (OSGPrimitive**) malloc(sizeof(OSGPrimitive*) * trace_length);
    for (int i=0; i<trace_length; i++){
      segments[i]=0;
    }
    // init lastpos with position of robot to follow
    Pos pos(robot->getPosition());
    lastpos=pos;

    counter=0;

    tracing_initialized=true;
  }



  void OdeAgent::step(double noise, double time){
    Agent::step(noise, time);
    trace();
    if(fixateRobot && !fixedJoint) tryFixateRobot();
  }

  void OdeAgent::beforeStep(GlobalData& global){
    OdeRobot* r = getRobot();
    r->sense(global);
    Operator::ManipDescr d;
    Operator::ManipType m;    
    FOREACH(OperatorList, operators, i){
      m=(*i)->observe(this, global, d);
      switch(m){
      case Operator::RemoveOperator:        
        delete *i;
        i=operators.erase(i);
        if(i!=operators.end()) i--;
        break;
      case Operator::Move:
        if(d.show){
          global.addTmpObject(new TmpDisplayItem(new OSGSphere(d.size.x()), 
                                                 TRANSM(d.pos), "manipmove"),0.5);
        }
        break;
      case Operator::Limit:
        if(d.show){
          global.addTmpObject(new TmpDisplayItem(new OSGCylinder(d.size.x(),d.size.z()), 
                                                 d.orientation * TRANSM(d.pos), 
                                                 "maniplimit", 0.2),0.5);
        }        
        break;
      default: break;
      }
    }
  }

  void OdeAgent::stepOnlyWiredController(double noise, double time) {
    WiredController::step(rsensors,rsensornumber, rmotors, rmotornumber, noise, time);
    trackrobot.track(robot, time); // we have to do this here because we agent.step is not called
    trace();
  }

  void OdeAgent::trace(){
    if (trackrobot.isDisplayTrace() && t%10==0){
      if (!tracing_initialized) {
	init_tracing();
      }
      Pos pos(robot->getPosition());
      /* draw cylinder only when length between actual
	 and last point is larger then a specific value
      */
      double len = (pos - lastpos).length();
      if(len > trace_thickness) {      
	if(segments[counter%trace_length]) delete segments[counter%trace_length];
	OSGPrimitive* s = new OSGCylinder(trace_thickness, len*1.2);
	s->init(((OdeRobot*)robot)->osgHandle, OSGPrimitive::Low);	
	s->setMatrix(osg::Matrix::rotate(osg::Vec3(0,0,1), (pos - lastpos)) *
		     osg::Matrix::translate(lastpos+(pos - lastpos)/2));
	segments[counter%trace_length] = s;
	lastpos = pos;
	counter++;
      }
    }
  }

  void OdeAgent::setMotorsGetSensors() {
    robot->setMotors(rmotors, rmotornumber);

    assert(robot && rsensors && rmotors);

    int len =  robot->getSensors(rsensors, rsensornumber);
    if(len != rsensornumber){
      fprintf(stderr, "%s:%i: Got not enough sensors, expected %i, got %i!\n", __FILE__, __LINE__,
	      rsensornumber, len);
    }
  }

  void OdeAgent::startMotorBabblingMode (int steps, AbstractController* babblecontroller, 
					 bool fixRobot){
    WiredController::startMotorBabblingMode(steps, babblecontroller, fixRobot);
    if(fixRobot){
      OdeRobot* r = dynamic_cast<OdeRobot*>(robot);
      if(!r) return;
      Primitive* main = r->getMainPrimitive();
      if(main){
	fixatingPos = main->getPosition();
	if(fixatingPos.z()< 1)
	  fixatingPos.z() += 1;    
	  fixateRobot = true;
      }    
    }
  }
    
  void OdeAgent::stopMotorBabblingMode (){
    WiredController::stopMotorBabblingMode();
    fixateRobot = false;
    if(fixedJoint)
      delete fixedJoint;
  }

  void OdeAgent::tryFixateRobot(){
    OdeRobot* r = dynamic_cast<OdeRobot*>(robot);
    if(!r) return;
    Primitive* main = r->getMainPrimitive();
    if(main){            
      Pos diff = fixatingPos-main->getPosition();;
      if(diff*diff<.1){
        // DummyPrimitive is a mem leak here.
        fixedJoint = new FixedJoint(new DummyPrimitive(),main);
        fixedJoint->init(r->odeHandle, r->osgHandle);
      }else{
        main->applyForce(diff*10); // use PID here
      }      
    }
    
  }

  bool OdeAgent::store(FILE* f) const {
    const OdeRobot* r = getRobot();
    return r->store(f) && getController()->store(f);    
  }

  bool OdeAgent::restore(FILE* f){
    OdeRobot* r = getRobot();
    return r->restore(f) && getController()->restore(f);        
  }


  
  void OdeAgent::addOperator(Operator* o, bool addToConfigurable){
    if(o){
      operators.push_back(o);
      if(addToConfigurable){
        addConfigurable(o);
      }
    }
    
  }

  bool OdeAgent::removeOperator(Operator* o){
    unsigned int size = operators.size();
    operators.remove(o);
    removeConfigurable(o); 
    return operators.size() < size;
  }
  
  void OdeAgent::removeOperators(){
    FOREACHC(OperatorList, operators, i){
      removeConfigurable(*i); 
      delete (*i);
    }
    operators.clear();
  }


}


