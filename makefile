CC          = c++ 

#-----------------------------------------
#Optimization ----------------------------
OPT   = -O3 -Wno-deprecated

GL_LIB = -framework OpenGL -framework GLUT -framework foundation
#GL_LIB = -lGL -lglut -lGLU

#-----------------------------------------

TARGETS = viewer fishtank

#-----------------------------------------

LIBS = 
INCS = -I/usr/local/include/eigen3 -I/usr/include/eigen3

CCOPTS = $(OPT) $(INCS) 
LDOPTS = $(OPT) $(INCS) $(LIBS) 

#-----------------------------------------
#-----------------------------------------

default: $(TARGETS)


clean:
	/bin/rm -f *.o $(TARGETS)

#-----------------------------------------
#-----------------------------------------

viewer: viewer.o
	$(CC) viewer.o $(LDOPTS) $(GL_LIB) -o viewer

fishtank: fishtank.o
	$(CC) fishtank.o $(LDOPTS) -o fishtank

#-----------------------------------------
#-----------------------------------------

.cpp.o: 
	$(CC) $(CCOPTS) -c $< 

#-----------------------------------------
#-----------------------------------------















