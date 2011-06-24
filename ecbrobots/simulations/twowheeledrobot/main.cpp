//#include <QApplication>
#include <qapplication.h>
#include "QECBRobotsWindow.h"
#include "QMessageDispatchWindow.h"
#include "QECBCommunicator.h"
#include "QGlobalData.h"
#include "ecb.h"

#include "commanddefs.h"

#include <selforg/sos.h>
#include <selforg/sox.h>
#include <selforg/sinecontroller.h>
#include <selforg/one2onewiring.h>
#include <selforg/abstractcontrolleradapter.h>
#include <selforg/measureadapter.h>
#include <selforg/oneactivemultipassivecontroller.h>
#include <selforg/mutualinformationcontroller.h>

using namespace lpzrobots;
using namespace std;

static int zeroRange = 0.00;
/**
 *
 */
class CheatedECB : public ECB {
  public:
    CheatedECB(QString dnsName, QGlobalData& globalData, ECBConfig& ecbConfig) :
      ECB(dnsName, globalData, ecbConfig) {
    }

    virtual ~CheatedECB() {
    }

    virtual void sendMotorValuesPackage() {
      // at first start of ECB, it must be initialized with a reset-command
      if (!(initialised && failureCounter <= globalData->maxFailures)) {
        sendResetECB();
        return;
      }

      globalData->textLog("ECB(" + dnsName + "): sendMotorPackage()!");

      // prepare the communication-protocol
      ECBCommunicationEvent* event = new ECBCommunicationEvent(
          ECBCommunicationEvent::EVENT_REQUEST_SEND_COMMAND_PACKAGE);

      event->commPackage.command = COMMAND_MOTORS;
      event->commPackage.dataLength = currentNumberMotors;
      // set motor-data
      int i = 0;
      // motorList was update by ECBAgent->ECBRobot:setMotors()->(to all ECBs)-> ECB:setMotors()
      FOREACH (list<motor>,motorList,m) {
        // Agent and Controller process with double-values
        // The ECB(hardware) has to work with byte-values
        if ((*m) >= -zeroRange && (*m) <= zeroRange)
          event->commPackage.data[i++] = convertToByte(0);
        else
          event->commPackage.data[i++] = convertToByte((*m));
      }
      informMediator(event);
    }

};

class MyController : public AbstractControllerAdapter {
  public:
    MyController(AbstractController* controller) :
      AbstractControllerAdapter(controller, "MyController", "$ID") {

    }

    virtual void step(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {
      controller->step(sensors, sensornumber, motors, motornumber);
      // motors[0] = 0;
      //motors[1] = -1;
    }
};

class MyECBManager : public QECBManager {

  public:

    Sos* myCon;
    ECBAgent* myAgent;
    Configurable::paramval c00;
    Configurable::paramval c01;
    Configurable::paramval c10;
    Configurable::paramval c11;
    Configurable::parambool showP;
    Configurable::parambool showF;
    Configurable::parambool showXsiF;


    MyECBManager(int argc, char** argv) :
      QECBManager(argc, argv) {
      QGlobalData& global = getGlobalData();

      // initial matrix values of C
      global.addParameterDef("C[0,0]",&c00, 1.1, -5, 5, "feedback strength of Matrix C");
      global.addParameterDef("C[0,1]",&c01, 0, -5, 5, "feedback strength of Matrix C");
      global.addParameterDef("C[1,0]",&c10, 0, -5, 5, "feedback strength of Matrix C");
      global.addParameterDef("C[1,1]",&c11, 1.1, -5, 5, "feedback strength of Matrix C");
      global.addParameterDef("showF", &showF, true, "display the frequency matrix (inspectable interface");
      global.addParameterDef("showP", &showP, false, "display the probability matrix (inspectable interface");
      global.addParameterDef("showXsiF", &showXsiF, false, "display the frequency matrix of H_xsi (inspectable interface");

    }

    virtual ~MyECBManager() {
    }

    /**
     * This function is for the initialisation of
     * ECBagents, ECBRobots and their heart, the Controller.
     * @param global
     * @return true if initialisation was successful
     */
    virtual bool start(QGlobalData& global) {

      // set specific communication values
      global.maxFailures = 4;
      global.serialReadTimeout = 100;
      global.cycleTime = 50;
      global.noise = 0.05;
      //global.plotOptions.push_back(PlotOption(GuiLogger, 1));

      int numberNimm2 = 3;
      for (int nimm2Index = 0; nimm2Index < numberNimm2; nimm2Index++) {
        if (nimm2Index != 1)
          continue;
        // create new controller

//        conConf.initialC = matrix::Matrix(2, 2);
//        conConf.initialC.val(0, 0) = 1.1;
//        conConf.initialC.val(1, 1) = 1.1;
//        conConf.initialC.val(1, 0) = -0.07;
//        conConf.initialC.val(0, 1) = -0.07;*/

          myCon = new Sos();
//        myCon = new SineController();

        // myCon->setName(QString("MyController"+QString::number(nimm2Index)).toStdString());
        //AbstractController* myCon = new SineController();
        //myCon->setParam("period", 500);

        // myCon->setParam("epsA", 0);
        //myCon->setParam("epsC", 0);

        // create new wiring
          myCon->setName("my cute Sos");

        AbstractWiring* myWiring = new One2OneWiring(new WhiteNormalNoise());
        // create new robot
        ECBRobot* myRobot = new ECBRobot(global);

        // create ECB
        ECBConfig ecbConf = ECB::getDefaultConf();
        ecbConf.maxNumberSensors = 2; // no infrared sensors
        ecbConf.maxNumberMotors = 2;
        QString* DNSName;
        switch (nimm2Index) {
          case 0:
            //            DNSName = new string("NIMM2_PRIMUS");
            DNSName = new QString("ECB_NIMM2_PRIMUS");
            break;
          case 1:
            DNSName = new QString("ECB_NIMM2_SECUNDUS");
            break;
          case 2:
            DNSName = new QString("ECB_NIMM2_TERTIUS");
            break;
          default:
            break;
        }
        ECB* myECB = new CheatedECB(*DNSName, global, ecbConf);
        myRobot->addECB(myECB);

        // create new agent
        list<PlotOption> plotList;
        //plotList.push_back(PlotOption(GuiLogger, 5));
//        plotList.push_back(PlotOption(MatrixViz, 5));
        plotList.push_back(PlotOption(File, 5));
        myAgent = new ECBAgent(plotList);

        OneActiveMultiPassiveController* onamupaco = new OneActiveMultiPassiveController(myCon);
//          mic = new MutualInformationController(1, -1, 1, true, true);
        MutualInformationController* mic = new MutualInformationController(30, -1, 1, showF, showP, showXsiF);
//        stats->addMeasure(mic->getH_yx(0), "H(x,y)[0]", ID, 0);
//        stats->addMeasure(mic->getH_yx(1), "H(x,y)[1]", ID, 0);
        MeasureAdapter* ma = new MeasureAdapter(mic);
        ma->getStatisticTools()->addMeasure(mic->getH_x(0), "H(x)[0]", ID, 0);
        ma->getStatisticTools()->addMeasure(mic->getH_x(1), "H(x)[1]", ID, 0);
        ma->getStatisticTools()->addMeasure(mic->getH_yx(0), "H(x,y)[0]", ID, 0);
        ma->getStatisticTools()->addMeasure(mic->getH_yx(1), "H(x,y)[1]", ID, 0);
        onamupaco->addPassiveController(ma);
        // init agent with controller, robot and wiring

        myAgent->preInit(onamupaco, myRobot, myWiring);

        // register agents
        global.agents.push_back(myAgent);
        global.configs.push_back(myAgent);

        delete DNSName;

      }

      return true;
    }


    /** optional additional callback function which is called when closed loop
     * is established (hardware ECBs, ECBAgent etc. are initialized)
     * To use this method, just overload it.
     * @param globalData The struct which contains all neccessary objects
     * like Agents
     * @agentInitialized the ECBAgent which is intialized
     */
    virtual void addCallbackAgentInitialized(QGlobalData& globalData, ECBAgent* agent) {
    }


    /** optional additional callback function which is called every
     * simulation step.
     * To use this method, just overload it.
     * @param globalData The struct which contains all neccessary objects
     * like Agents
     * @param paused indicates that simulation is paused
     * @param control indicates that robots have been controlled this timestep (default: true)
     */
    virtual void addCallbackStep(QGlobalData& globalData, bool pause, bool control) {
      if (myAgent->isInitialized()) {
        matrix::Matrix initialC(2,2);
        initialC.val(0,0) = c00;
        initialC.val(0,1) = c01;
        initialC.val(1,0) = c10;
        initialC.val(1,1) = c11;
        myCon->setC(initialC);
      }
      if (globalData.controlStep == 20 * 60 * 10 +1)
        stopLoop();
    }


    /** add own key handling stuff here, just insert some case values
     *
     * @param globalData
     * @param key
     * @return
     */
    virtual bool command(QGlobalData& globalData, int key) {
      return false;
    }

};

/**
 * normally here do not change anything
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char *argv[]) {

  // create your custom ECBManager
  MyECBManager ecbManager(argc, argv);

  Q_INIT_RESOURCE(ecbrobots);

  QApplication app(argc, argv);

  QString appPath = QString(argv[0]);
  QECBRobotsWindow *ecbWindow = new QECBRobotsWindow(appPath.mid(0, appPath.lastIndexOf("/") + 1), &ecbManager);
  QMessageDispatchWindow *messageDispatchWindow = new QMessageDispatchWindow(appPath.mid(0, appPath.lastIndexOf("/")
      + 1));

  qRegisterMetaType<struct _communicationMessage> ("_communicationMessage");
  QObject::connect(ecbManager.getGlobalData().comm, SIGNAL(sig_sendMessage(struct _communicationMessage)),
      messageDispatchWindow->getQMessageDispatchServer(), SLOT(sl_sendMessage(struct _communicationMessage)));
  QObject::connect(messageDispatchWindow->getQMessageDispatchServer(),
      SIGNAL(sig_messageReceived(struct _communicationMessage)), ecbManager.getGlobalData().comm,
      SLOT(sl_messageReceived(struct _communicationMessage)));
  QObject::connect(messageDispatchWindow->getQMessageDispatchServer(), SIGNAL(sig_quitServer()),
      ecbManager.getGlobalData().comm, SLOT(sl_quitServer()));
  QObject::connect(ecbManager.getGlobalData().comm, SIGNAL(sig_quitClient()),
      messageDispatchWindow->getQMessageDispatchServer(), SLOT(sl_quitClient()));

  messageDispatchWindow->show();
  ecbWindow->show();

  return app.exec();
}

