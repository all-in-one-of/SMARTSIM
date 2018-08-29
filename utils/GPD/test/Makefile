#!/bin/make -f
#

APPNAME = gpd

#-----------------------------------------------------------------------

CXXFILES = GPD_Main.C

OBJECTS=$(CXXFILES:.C=.o)

CXXFLAGS = -I.. -g

CXX = g++

%.o : %.C
	$(CXX) -c $(CXXFLAGS) $< -o $@

LDFLAGS = -L.. \
	-lGPD



#-----------------------------------------------------------------------

default:	$(APPNAME) makesphere geoinfo

$(APPNAME):	$(OBJECTS)
	$(CXX) -o $@ $(OBJECTS) $(LDFLAGS)

makesphere:	GPD_MakeSphere.o
	$(CXX) -o $@ GPD_MakeSphere.o $(LDFLAGS)

geoinfo:	GPD_GeoInfo.o
	$(CXX) -o $@ GPD_GeoInfo.o $(LDFLAGS)

clean:
	rm -f *.o a.out core

clobber:	clean
	rm -f $(APPNAME)
	rm -f makesphere

install:

production:
