# define variables
HDRDIR  = ./

# set options for this machine
# specify which compilers to use for c and linking
CC	= g++
LD	= g++

# compiler flags to be used (set to compile with debugging on)
CFLAGS = -I$(HDRDIR)  -Ddfloat=double -DdfloatString='"%lg"' 

# link flags to be used 
LDFLAGS	= 

# libraries to be linked in
LIBS	=  -lm 

# types of files we are going to construct rules for
.SUFFIXES: .c .cu

# rule for .c files
.c.o:
	$(CC) $(CFLAGS) -o $*.o -c $*.c

# list of objects to be compiled
SOBJS = src/sensor.o src/utils.o src/grid.o src/saveppm.o src/sceneSetup.o src/readPlyModel.o  src/simpleRayTracer.o  src/intersectionTests.o src/shape.o src/projectionTests.o src/boundingBoxes.o src/render.o src/sphereDynamics.o

all: simpleRayTracer

simpleRayTracer:$(SOBJS) 
	$(LD)  $(LDFLAGS) -o simpleRayTracer $(SOBJS) $(LIBS)

# what to do if user types "make clean"
clean :
	rm -r $(SOBJS) simpleRayTracer 

realclean :
	rm -r $(SOBJS) simpleRayTracer images/*.ppm images/*.png images/*.mp4 


