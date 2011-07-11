#include <configurator/ConfiguratorProxy.h>

#include <selforg/sox.h>
#include <selforg/soxexpand.h>

#include <iostream>

using namespace lpzrobots;
using namespace std;

/**
 * normally here do not change anything
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char *argv[]) {

  Configurable::configurableList configList;
  Configurable::paramint activeController;
  Configurable::paramval dummyParam;

  Configurable* agent = new Configurable("DummyConfigAgent","rev0");
  agent->addParameterDef("activeController", &activeController, 0, 0, 1, "Switches between the Controllers to be active.");
  Configurable* robot = new Configurable("DummyConfigRobot","rev0");
  robot->addParameterDef("dummyParameter", &dummyParam, 1, -2, 2, "This is a dummy parameter who has no function.");
  Sox* sox1 = new Sox();
  SoxExpand* sox2 = new SoxExpand();
  sox1->init(1,1,0);
  sox2->init(1,1,0);

  configList.push_back(agent);
  agent->addConfigurable(robot);
  agent->addConfigurable(sox1);
  agent->addConfigurable(sox2);

  ConfiguratorProxy proxy(argc, argv, configList);
  std::cout << "ConfiguratorProxy created." << std::endl;
}

