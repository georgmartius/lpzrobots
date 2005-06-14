#ifndef _PLOTLOG_H_
#define _PLOTLOG_H_

#include "gnuplot.h"
#include "logfile.h"

/// \ingroup baseclasses

/** gnuplot and logfile classes in one. if you need to print and plot the same data, you can use this class */ 

template <class T> class PlotLog :  public Gnuplot<T> , public LogFile<T>
{
 public:
    PlotLog(int buffersize,const std::string& filename="",const std::string& info="",const std::string& separator="\t")
	:Gnuplot<T>(buffersize)
	,LogFile<T>(filename,info,separator){
    };

    PlotLog(const std::string& filename="",const std::string& info="",const std::string& separator="\t")
	:Gnuplot<T>(256)
	,LogFile<T>(filename,info,separator){
    };

/// add a new channel    
    void addChannel(const T& name , const std::string& title="",const std::string& style="lines"){	
	Gnuplot<T>::addChannel(name,title,style);
	LogFile<T>::addChannel(name);
    };

/// put data in buffer
    void putData(const T& channel, double value){
	Gnuplot<T>::putData(channel,value);
	LogFile<T>::putData(channel,value);
    }; 
};

#endif
