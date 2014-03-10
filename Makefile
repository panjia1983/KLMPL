OBJDIR = objs

#################### Start the action #########################
.PHONY: all test clean docs lib

all:
	cd misc; make
	cd MotionPlanning; make
	cd example; make all
	ar -rcs LMPL.a $(wildcard misc/objs/*.o) $(wildcard MotionPlanning/objs/*.o)
	ranlib LMPL.a
	doxygen doxygen.conf

test:
	cd example; make all

lib:
	cd misc; make
	cd MotionPlanning; make
	ar -rcs LMPL.a $(wildcard misc/objs/*.o) $(wildcard MotionPlanning/objs/*.o)
	ranlib LMPL.a

docs:
	doxygen doxygen.conf

clean:
	rm -rf *.bak *.bac Debug Release
	rm doxygen_warnings.log
	rm -r docs
	rm LMPL.a
	cd misc; make clean 
	cd MotionPlanning; make clean
	cd example; make clean


