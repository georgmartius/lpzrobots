

#ifndef __SRGUI_PIPE_FILTER_H__
#define __SRGUI_PIPE_FILTER_H__

#include "AbstractPipeFilter.h"
#include "AbstractPlotChannel.h"
#include "IRPlotChannel.h"
#include "MotorCurrentPlotChannel.h"
#include "MotorSpeedPlotChannel.h"
#include "TiltPlotChannel.h"
#include "AxesPlotChannel.h"
#include "TimeStampPlotChannel.h"
#include "DefaultPlotChannel.h"

#include <string>


class SRGUIPipeFilter : public AbstractPipeFilter
{

  Q_OBJECT

public:
  SRGUIPipeFilter(AbstractPipeReader* apr) : AbstractPipeFilter(apr)
  {
    std::cout << "new SRGUIPipeFilter()" << std::endl;
  };

  virtual ~SRGUIPipeFilter() {};

  /**
  * The dataLine from PipeReader will be iterate to set the new channel-value.
  * The order of the value-input-list (dataList) is important and the index of it
  * must be equal to the indexnumber of the private channelIndexList that was created by
  * createChannelList().
  *
  * If a new sensor is plugged into hardware-ECB, a new descriptionLine will be created and
  * the PipeFilter must be reinit to reorder the channelIndexList
  */
  virtual void updateChannels()
  {
//     std::cout << "AbstractPipeFilter: updateChannels()" << std::endl;

//     std::cout << "AbstractPipeFilter: updateChannels(";

    std::list<double> dataList = (apr->getDataLine());
    int index=0;
    std::list<int>::const_iterator index_it=channelIndexList.begin();
    std::list<AbstractPlotChannel*>::const_iterator channel_it=channelList.begin();

    int tmp_i=0;
    for(std::list<double>::iterator i=dataList.begin(); i != dataList.end(); i++) {
      printf("[% .1f]",(*i));
      if (tmp_i > 5) break;
      tmp_i++;
    }
    printf("\r\n");

    int printedIndex = 0;

    for(std::list<double>::iterator i=dataList.begin(); i != dataList.end() && index_it!=channelIndexList.end() && channel_it!=channelList.end() ; i++)
    {
      if (index == (*index_it))
      {
//         std::cout << "[" << (*channel_it)->getChannelName() << "=" << index << "]";

        if ( ((*i) <= 1.) && ((*i) >= -1.) ) {
          (*channel_it)->setValue((*i));

          if (printedIndex < 7) {
            printf("[ %3d]",index);
            printedIndex++;
          }
        }
        else //the old value has to be
          printf("[old~]");

        channel_it++;
        index_it++;
      }
//       else std::cout << "[  - ]";

      index++;
    }
//     std::cout << ")" << std::endl;
    printf("\r\n");
   }

protected:
  /**
  * Namen k�nnen sein:
  t x[0] x[1] x[2] x[3] y[0] y[1] y[2] y[3] A[0,0] A[0,1] A[0,2] A[0,3] A[1,0] A[1,1] A[1,2] A[1,3] A[2,0] A[2,1] A[2,2] A[2,3] A[3,0] A[3,1] A[3,2] A[3,3] C[0,0] C[0,1] C[0,2] C[0,3] C[1,0] C[1,1] C[1,2] C[1,3] C[2,0] C[2,1] C[2,2] C[2,3] C[3,0] C[3,1] C[3,2] C[3,3] R[0,0] R[0,1] R[0,2] R[0,3] R[1,0] R[1,1] R[1,2] R[1,3] R[2,0] R[2,1] R[2,2] R[2,3] R[3,0] R[3,1] R[3,2] R[3,3] H[0] H[1] H[2] H[3] B[0] B[1] B[2] B[3]
  z.B. f�r Tiltsensoren (bsp. 4 St�ck)
  tilt[0]...tilt[3]
  generell gilt: x[..] motor undy[..] sensor
  Aber: Name und Adresse mit addParameterDef(..)(Inspectable) durch ECB anmelden
  */

  /**
  * Function is called from AbstractPipeFilter and need the names of descriptionLine from
  * ECB to create a new channel. Every filter must override this fuction to user-defined-filtering
  * After creation of channels, the AbstractPipeFilter will update this values automaticly and a
  * GUI will read this values for printing.
  */

  AbstractPlotChannel* createChannel(std::string name)
  {
//     std::cout << "SRGUIPipeFilter: createChannel(" << name << ")" << std::endl;

    if (name.find("y[0]")==0) return (new MotorSpeedPlotChannel("motorCspeedX"));
    if (name.find("y[1]")==0) return (new MotorSpeedPlotChannel("motorCspeedY"));

    if (name.find("ms0_0(1)")==0) return (new MotorSpeedPlotChannel("motorRspeedX"));
    if (name.find("ms0_1(1)")==0) return (new MotorSpeedPlotChannel("motorRspeedY"));
    if (name.find("mc0_0(1)")==0) return (new MotorCurrentPlotChannel("motorcurrentX"));
    if (name.find("mc0_1(1)")==0) return (new MotorCurrentPlotChannel("motorcurrentY"));

    if (name.find("adc2(1)")==0) return (new TiltPlotChannel("tiltX"));
    if (name.find("adc3(1)")==0) return (new TiltPlotChannel("tiltY"));
    if (name.find("adc0(1)")==0) return (new AxesPlotChannel("AxesWatcher_left"));
    if (name.find("adc1(1)")==0) return (new AxesPlotChannel("AxesWatcher_rigth"));

    if (name.find("pcf0_0(2)")==0) return (new IRPlotChannel("ir0"));
    if (name.find("pcf0_1(2)")==0) return (new IRPlotChannel("ir1"));
    if (name.find("pcf0_2(2)")==0) return (new IRPlotChannel("ir2"));

    if (name.find("pcf1_0(2)")==0) return (new IRPlotChannel("ir3"));
    if (name.find("pcf1_1(2)")==0) return (new IRPlotChannel("ir4"));
    if (name.find("pcf1_3(2)")==0) return (new IRPlotChannel("ir5"));

    if (name.find("pcf3_0(2)")==0) return (new IRPlotChannel("ir6"));
    if (name.find("pcf3_1(2)")==0) return (new IRPlotChannel("ir7"));
    if (name.find("pcf3_2(2)")==0) return (new IRPlotChannel("ir8"));

    if (name.find("pcf7_2(2)")==0) return (new IRPlotChannel("ir9"));
    if (name.find("pcf7_3(2)")==0) return (new IRPlotChannel("ir10"));

//     if (name.find("timestamp")==0) return (new TimeStampPlotChannel("step"));

    return 0;
  }

private:

};

#endif
