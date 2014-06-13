#ifndef __ABSTRACT_PIPE_FILTER_H__
#define __ABSTRACT_PIPE_FILTER_H__

#include <list>

#include <QObject>

#include "AbstractPipeReader.h"
#include "AbstractPlotChannel.h"
#include "TimeStampPlotChannel.h"

#include "sensortypes.h"

// class AbstractPipeReader;
// class AbstractPlotChannel;

class AbstractPipeFilter : public QObject {

    Q_OBJECT

    public:

    AbstractPipeFilter(AbstractPipeReader* apr) : apr(apr)
    {
    };
    virtual ~AbstractPipeFilter() {
    }
    ;

    virtual std::list<AbstractPlotChannel*> getChannelList() {
      //     if (apr->getChannelLine()
      if (channelList.empty())
        createChannelList();
      return channelList;
    }

    virtual AbstractPipeReader* getPipeReader() {
      return this->apr;
    }

    virtual void setPipeReader(AbstractPipeReader* apr) {
      this->apr = apr;
      channelList.clear();
      channelIndexList.clear();
    }

  class MySleep : public QThread
  {
  public:
    static void msleep(unsigned long msecs) {
      QThread::msleep(msecs);
    }
  };

  public slots:
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

    for(std::list<double>::iterator i=dataList.begin(); i != dataList.end() && index_it!=channelIndexList.end() && channel_it!=channelList.end(); i++)
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
    std::list<int> channelIndexList;
    std::list<AbstractPlotChannel*> channelList;
    AbstractPipeReader* apr;

    virtual void createChannelList() {
      if(debug) std::cout << "AbstractPipeFilter: createChannelList()" << std::endl;

      while (!(apr->readyForData())) {
        MySleep::msleep(50);
      }

      std::list<std::string> tmp_list;
      std::list<std::string> description_list;
      std::list<std::string> channel_list;

      channel_list = (apr->getChannelLine());
      description_list = apr->getDescriptionLine();

      std::list<std::string>::iterator it_channel = channel_list.begin();

      // first element of comming dataLine is always the timestamp
      tmp_list.push_back("timestamp");
      // channel iterator must be increment to jump over this timestamp
      it_channel++;

      if(debug) std::cout << "tmp_list: [timestamp,";

      for (std::list<std::string>::iterator it_description = description_list.begin(); it_description
          != description_list.end(); it_description++) {
        if(debug) printf("%s,", (*it_description).c_str());
        tmp_list.push_back((*it_description));
        it_channel++;
      }

      for (; it_channel != channel_list.end(); it_channel++) {
        if(debug) printf("%s,", (*it_channel).c_str());
        tmp_list.push_back((*it_channel));
      }

      if(debug) std::cout << "]" << std::endl;

      // jump to second position
      //    it++;
      /*
       for (std::list<std::string>::iterator i=tmp_list2.begin(); i!=tmp_list2.end(); i++) {

       it = tmp_list.erase(it);
       it = tmp_list.insert(it, (*i));
       if (it != tmp_list.end()) it++;

       }
       */
      //     printf("PipeFilter: [");
      //     for(std::list<std::string>::iterator i=tmp_list.begin(); i != tmp_list.end(); i++)
      //     {
      //       printf("%s-",(*i).c_str());
      //
      //     }
      //     printf("]\r\n");
      //
      //     // timestamp will exists every time
      //     // add the TimeStampPlotChannel at front of channelList
      // //     channelList.push_back((AbstractPlotChannel*)(new TimeStampPlotChannel("TimeStamp")));
      // //     channelIndexList.push_back(0);
      //
      //further channels will be add here...


      if(debug) std::cout << "(" << std::endl;
      int index = 0;
      for (std::list<std::string>::iterator i = tmp_list.begin(); i != tmp_list.end(); i++) {
        AbstractPlotChannel* newChannel = createChannel((*i));
        // not all channels are in the process of interests
        if (newChannel != 0) {
          channelList.push_back(newChannel);
          channelIndexList.push_back(index);
          if(debug) std::cout << "+[" << newChannel->getChannelName() << "]";
        } else
          if(debug) std::cout << "-[" << (*i) << "]";

        index++;
      }

      if(debug) std::cout << ")" << std::endl;
    }

    virtual AbstractPlotChannel* createChannel(std::string name) = 0;

  private:
    static const bool debug = false;

};

#endif
