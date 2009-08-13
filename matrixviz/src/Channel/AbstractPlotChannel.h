
#ifndef __ABSTRACT_PLOT_CHANNEL_H__
#define __ABSTRACT_PLOT_CHANNEL_H__

#include <string>

class AbstractPlotChannel
{
public:
  AbstractPlotChannel(std::string name) : name(name), channelValue(0) {};
//   virtual ~AbstractPlotChannel() {};
  
  virtual void setValue(double v) { channelValue=v; }
  
  virtual double getValue()       { return channelValue; }
  
  virtual std::string getChannelName() { return name; }  
  
    
protected:
  
  
private:
  
  std::string name;
  double channelValue;
  
};

#endif
