#File:     Makefile for lpzrobot directory
#Author:   Georg Martius  <martius@informatik.uni-leipzig.de>
#Date:     June 2005

include Makefile.conf

all: $(SIMULATIONS)

.PHONY: clean $(SIMULATIONS)
$(SIMULATIONS): 
	@echo -e "\n************ Building SIMULATION $@ **************"
	$(MAKE) -C $@

clean: 
	for dir in $(SIMULATIONS); do \
		$(MAKE) -C $$dir clean; \
	done



