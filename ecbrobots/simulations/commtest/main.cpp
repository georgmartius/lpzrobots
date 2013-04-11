/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    marcoeckert@web.de                                                   *
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
 *   Revision 1.4  2010-11-14 20:37:02  wrabe
 *   - change object-name from 'comm' to 'commchannel' to avoid mix-up
 *
 *   Revision 1.3  2010/11/10 09:31:24  guettler
 *   - port to Qt part 1
 *
 *   Revision 1.2  2008/07/16 14:37:17  robot1
 *   -simple getc included
 *   -extended config on reset
 *   -minor changes
 *
 *   Revision 1.1  2008/07/16 07:38:42  robot1
 *   some major improvements
 *
 *   Revision 1.3  2008/04/11 06:31:16  guettler
 *   Included all classes of ecbrobots into the namespace lpzrobots
 *
 *   Revision 1.2  2008/04/08 10:11:03  guettler
 *   alpha testing
 *
 *   Revision 1.1.1.1  2008/04/08 08:14:30  guettler
 *   new ecbrobots module!
 *
 *                                                                         *
 ***************************************************************************/
#include "ecbmanager.h"
#include "ecbcommunicator.h"

#include <selforg/invertmotorspace.h>
#include <selforg/one2onewiring.h>
#include "commanddefs.h"


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

class ECBTestCommunicator : public ECBCommunicator {
public:
//         GlobalData* global;

        ECBTestCommunicator(GlobalData& global) : ECBCommunicator(global) {}

        virtual ~ECBTestCommunicator(){}

        virtual bool testModeCallback() {
          commData data;
          data.destinationAddress = 1;
          data.sourceAddress = 0;
          data.command = CPING; //CCOMTEST
          data.dataLength = 1;
          data.data[0] = 23;
          if (!globalData->commchannel->sendData(data)) {
                cerr << "Error while sending motor values for ECB " << 1 << "." << endl;
          }
          commData result = globalData->commchannel->receiveData();
        printf("command(hex): %x,\r\n",result.command);
          for(int i=0;i<result.dataLength;i++) {
                    std::cout << "result: " << result.data[i] << std::endl;
            }


        return true;
        }

};

class MyECBManager : public ECBManager {

  public:

    MyECBManager(ECBCommunicator* comm) : ECBManager(comm) {}

    virtual ~MyECBManager() {}

    /**
     * This function is for the initialising of agents and robots,...
     * @param global
     * @return true if initialisation was successful
     */
    virtual bool start ( GlobalData& global ) {

      // set specific communication values
      global.baudrate = 115200;
      global.portName = "/dev/ttyS0";
      global.masterAddress=0;
      global.maxFailures=4;
      global.serialReadTimeout=50;
      global.verbose = true;
        global.testMode= true;

        global.debug = true;


      return true;
    }

    /** optional additional callback function which is called every simulation step.
    Called between physical simulation step and drawing.
    @param paused indicates that simulation is paused
    @param control indicates that robots have been controlled this timestep
    */
    virtual void addCallback ( GlobalData& globalData,bool pause, bool control ) {

    }

    /** add own key handling stuff here, just insert some case values
     *
     * @param globalData
     * @param key
     * @return
     */
    virtual bool command ( GlobalData& globalData, int key) {
      return false;
    }


};


/**
 * normally here do not change anything
 * @param argc
 * @param argv
 * @return
 */
int main ( int argc, char **argv ) {
  GlobalData global;
  MyECBManager* ecb = new MyECBManager(new ECBTestCommunicator(global));
  return ecb->run ( argc, argv ) ? 0 : 1;
}



