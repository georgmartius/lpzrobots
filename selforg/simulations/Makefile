#File:     Makefile for selforg simulations directory
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     Mai 2008

include Makefile.conf

all: $(SIMULATIONS)

.PHONY: depend clean $(SIMULATIONS)

$(SIMULATIONS): 
	@echo -e "\n************ Building SIMULATION $@ **************"
	$(MAKE) -C $@

depend: 
	for dir in $(SIMULATIONS); do \
		$(MAKE) -C $$dir depend; \
	done
    

clean: 
	for dir in $(SIMULATIONS); do \
		$(MAKE) -C $$dir clean; \
	done



