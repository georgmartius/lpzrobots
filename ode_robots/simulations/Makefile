#File:     Makefile for lpzrobot directory
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     June 2005

# subdirectories which contain libraries or binaries needed some of the other project in this tree 
SIMULATIONS= atomsimulation jointtest pseudosensor_wheelrobot \
	  running_through_limit_cycles short_circuit \
	  template_onerobot template_schlange template_schlangeforce

all: $(SIMULATIONS)

.PHONY: $(SIMULATIONS)
$(SIMULATIONS): 
	@echo -e "\n************ Building SIMULATION $@ **************"
	$(MAKE) -C $@




