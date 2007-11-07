/*
** sound.h
** 
** Made by Georg Martius <georg@nld.ds.mpg.de>
** 
** Started on  Mon Oct 15 16:20:00 2007 Georg Martius
** Last update Mon Oct 15 16:20:00 2007 Georg Martius
*/

#ifndef   	SOUND_H_
# define   	SOUND_H_

#include "pos.h"

namespace lpzrobots {

  class OSGSphere;
  class OsgHandle;
  
  /// Object that represents a sound signal in the simulator
  class Sound {
  public:
    Sound(double time, const Pos& pos, float intensity, float frequency, void* sender)
      : time(time), pos(pos), 
      intensity(intensity), frequency(frequency), sender(sender),
      visual(0) {}

    ~Sound();
    
    /// nice predicate function for finding old sound signals
    struct older_than : public std::unary_function<const Sound&, bool> {
      older_than(double time) : time(time) {}
      double time;
      bool operator()(const Sound& s) { return s.time < time; }
    };

    void render(const OsgHandle& osgHandle);

    double time;
    Pos pos;    ///< emission position
    float intensity; ///< intensity -1..1
    float frequency; ///< frequency -1..1
    void* sender;    ///< pointer to the sender (can be used for self
		     ///detection)
    
  private:
    OSGSphere* visual;    
  };

} // end namespace

#endif 	    /* !SOUND_H_ */
