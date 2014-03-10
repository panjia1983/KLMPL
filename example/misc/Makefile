INCDIR= ../ ../../ GLdraw
include ../../Makefile.config

SRCS:=$(SRCS) $(wildcard GLdraw/*.cpp)

include ../../Makefile.template


.PHONY:  all GLdraw

all: GLdraw default
#	cd GLdraw;
	@echo $(SRCS_EX)	


GLdraw:
#	@echo $(SRCS)
	mkdir -p $(OBJDIR)/GLdraw;
	
clean: clean_template