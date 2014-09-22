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
#include <selforg/matrix.h>

namespace lpzrobots {

  void TraceDrawer::init(){
    assert(obj);
    lastpos = obj->getPosition();
    initialized=true;
  }

  void TraceDrawer::close(){
    if(initialized)
      tracker.close();
    initialized=false;
  }

  void TraceDrawer::track(double time){
    if (initialized){
      tracker.track(obj, time);
    }
  }

  void TraceDrawer::drawTrace(GlobalData& global){
    if (initialized && tracker.isDisplayTrace()){
      Position pos(obj->getPosition());
      double len = (pos - lastpos).length();
      if(tracker.conf.displayTraceThickness>0){ // use a cylinder
        /* draw cylinder only when length between actual
           and last point is larger then a specific value
        */
        if(len > 2*tracker.conf.displayTraceThickness) {
          global.addTmpObject(new TmpDisplayItem(new OSGCylinder(tracker.conf.displayTraceThickness, len*1.2),
                                                 ROTM(osg::Vec3(0,0,1), Pos(pos - lastpos)) *
                                                 TRANSM(Pos(lastpos)+Pos(pos - lastpos)/2),
                                                 color, OSGPrimitive::Low),
                              tracker.conf.displayTraceDur);
          lastpos = pos;
        }
      }else{ // use a line
        if(len > 0.05) {
          pnts.push_back(Pos(lastpos));
          pnts.push_back(Pos(pos));
          if(pnts.size()>16){
            global.addTmpObject(new TmpDisplayItem(new OSGLine(pnts), TRANSM(0,0,0), color),
                                tracker.conf.displayTraceDur);
            pnts.clear();
          }
          lastpos = pos;
        }
      }
    }
  }


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
    FOREACH(TraceDrawerList, segmentTracking, td){
      td->close();
    }

  }

  void OdeAgent::constructor_helper(const GlobalData* globalData){
    if(globalData){
      FOREACHC(std::list<Configurable*>, globalData->globalconfigurables, c){
        plotEngine.addConfigurable(*c);
      }
    }
  }

  void OdeAgent::step(double noise, double time){
    Agent::step(noise, time);
    // for the main trace we do not call track, this in done in agent
    // track the segments
    FOREACH(TraceDrawerList, segmentTracking, td){
      td->track(time);
    }
  }

  void OdeAgent::beforeStep(GlobalData& global){
    OdeRobot* r = getRobot();
    r->sense(global);

    trace(global);

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
                                                 TRANSM(d.pos), "manipmove"),
                              global.odeConfig.simStepSize*5);
          if(d.show>1)
            global.addTmpObject(new TmpDisplayItem(new OSGLine({d.posStart,d.pos}),
                                                   TRANSM(0,0,0), "manipmove"),
                                global.odeConfig.simStepSize
                                );
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
    trackrobot.track(robot, time); // we have to do this here because agent.step is not called
    // track the segments
    FOREACH(TraceDrawerList, segmentTracking, td){
      td->track(time);
    }

  }

  void OdeAgent::trace(GlobalData& global){
    mainTrace.drawTrace(global);
    FOREACH(TraceDrawerList, segmentTracking, td){
      td->drawTrace(global);
    }
  }

  void OdeAgent::setTrackOptions(const TrackRobot& trackrobot){
    Agent::setTrackOptions(trackrobot);
    if (trackrobot.isDisplayTrace()){
      mainTrace.obj=robot;
      mainTrace.tracker = trackrobot;
      mainTrace.color = ((OdeRobot*)robot)->osgHandle.color;
      mainTrace.init();
    }
  }

  bool OdeAgent::stopTracking(){
    bool rv=Agent::stopTracking();
    // we also want to stop the trace:
    mainTrace.close();

    return rv;

  }

  class TrackablePrimitive : public Trackable {
  public:
    TrackablePrimitive(Primitive* p, const std::string& name)
      : p(p), name(name) { }
    virtual std::string getTrackableName() const { return name; };
    virtual Position getPosition() const  { return p->getPosition().toPosition(); };
    virtual Position getSpeed() const     { return p->getVel().toPosition(); };
    virtual Position getAngularSpeed() const { return p->getAngularVel().toPosition(); };
    virtual matrix::Matrix getOrientation() const {
      fprintf(stderr, "TrackablePrimitive:: getOrientation(): not implemented\n");
      return matrix::Matrix(3,3);
    };

  protected:
    Primitive* p;
    std::string name;
  };

  /// adds tracking for individual primitives
  void OdeAgent::addTracking(unsigned int primitiveIndex,const TrackRobot& trackrobot,
                             const Color& color){
    assert(robot);
    TraceDrawer td;
    Primitives ps = ((OdeRobot*)robot)->getAllPrimitives();
    if(primitiveIndex >= ps.size()){
      fprintf(stderr, "OdeAgent::addTracking(): primitive index out of bounds %ui", primitiveIndex);
      return;
    }
    td.obj=new TrackablePrimitive(ps[primitiveIndex],
                                  ((OdeRobot*)robot)->getName() + "segm_" + std::itos(primitiveIndex));
    td.tracker = trackrobot;
    td.tracker.conf.id=primitiveIndex;
    td.color = color;
    td.init();
    if(!td.tracker.open(robot)){
      fprintf(stderr, "OdeAgent.cpp() ERROR: could not open trackfile! <<<<<<<<<<<<<\n");
    }
    segmentTracking.push_back(td);
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

  void OdeAgent::fixateRobot(GlobalData& global, int primitiveID, double time){
    OdeRobot* r = dynamic_cast<OdeRobot*>(robot);
    if(!r) return;
    r->fixate(global,primitiveID,time);
  }

  bool OdeAgent::unfixateRobot(GlobalData& global){
    OdeRobot* r = dynamic_cast<OdeRobot*>(robot);
    if(!r) return false;
    return r->unFixate(global);
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
