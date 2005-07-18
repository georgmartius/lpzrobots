#File:     Makefile for lpzrobot directory
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     June 2005

# subdirectories which contain libraries or binaries needed some of the other project in this tree 
SIMULATIONS= \
	jointtest \
	pseudosensor_wheelrobot \
	running_through_limit_cycles \
	short_circuit \
	template_onerobot \
	template_schlange \
	template_schlangeforce \
	atomsimulation

all: $(SIMULATIONS)

.PHONY: clean $(SIMULATIONS)
$(SIMULATIONS): 
	@echo -e "\n************ Building SIMULATION $@ **************"
	$(MAKE) -C $@

clean: 
	for dir in $(SIMULATIONS); do \
		$(MAKE) -C $$dir clean; \
	done



