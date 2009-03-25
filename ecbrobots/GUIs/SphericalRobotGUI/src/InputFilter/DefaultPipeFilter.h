

#ifndef __DEFAULT_PIPE_CHANNEL_H__
#define __DEFAULT_PIPE_CHANNEL_H__

#include "AbstractPipeFilter.h"

class DefaultPipeChannel
{
public:
  
  DefaultPipeChannel(AbstractPipeFilter* apr) : AbstractPipeFilter(apr) {};
  virtual ~DefaultPipeChannel();
  
protected:
  
  virtual AbstractChannel* createChannel(std::string name) 
  {
    return new DefaultPlotChannel(name);
  }
  
private:
  
  
};

#endif
