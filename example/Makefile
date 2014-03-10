.PHONY : all depend clean link

INCDIR= . ../

define RECUR
  FILES:=$(wildcard $(1)*)
  DIRS=
  $$(foreach e, $$(FILES),  $$(if $$(wildcard $$(e)/*), $$(eval DIRS=$$(DIRS) $$(e))))
  FILES:=$$(filter-out $$(DIRS),$$(FILES))
  FILES:=$$(filter $(2),$$(FILES))
  RESULT:=$$(RESULT) $$(FILES)
  $$(foreach e,$$(DIRS),$$(eval $$(call RECUR,$$(e)/, $(2))))
endef

#recursively get all .o files

#self-level configuration
include Makefile.config

#root-level configuration
include ../Makefile.config
include ../Makefile.template



#ifeq ($(BUILDNAME),linux)
#  LIB={UNSPECIFIED}
#endif

#ifeq ($(BUILDNAME),Windows_NT)     
#  LIB=$(PROGRAMFILES)
#  LIB:="/cygdrive/$(subst :,/,$(LIB))"
#endif


all: depend default 
	@echo $(CPPFLAGS)
	make link

depend:
	cd misc; make all;

clean: clean_template
	cd misc; make clean;

link:  
	-rm test
	$(eval $(call RECUR,../, %.o))
#	@echo ~/lib
	$(CC) $(RESULT) -o test -lm
	
t1:
	-rm test.o
	g++ -I/usr/include -c test.cpp
	g++ test.o -L/lib -lopengl32
	
t2: 
	


	
	


