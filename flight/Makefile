# Compiler, tools and options

FCLPATH  = /home/tck/AutoPilot

CC    			= gcc
CXX   			= g++ -std=c++11
DEFINES       	=
CFLAGS			= -pipe -O2 -Wall -W $(DEFINES)
CXXFLAGS      	= -pipe -O2 -Wall -W $(DEFINES)
INCPATH       	= -I. -I$(FCLPATH)
LINK  			= g++
LFLAGS			= -Wl,-O1
LIBS  			= -L/usr/lib/arm-linux-gnueabihf /usr/lib/libmraa.so /usr/lib/libupm-pca9685.so /usr/local/lib/librealsense.so
COPY  			= cp -f
COPY_FILE     	= $(COPY)
COPY_DIR      	= $(COPY) -r
STRIP 			= strip
INSTALL_FILE  	= install -m 644 -p
INSTALL_DIR   	= $(COPY_DIR)
INSTALL_PROGRAM = install -m 755 -p
DEL_FILE      	= rm -f
SYMLINK       	= ln -f -s
DEL_DIR       	= rmdir
MOVE  			= mv -f
CHK_DIR_EXISTS	= test -d
MKDIR			= mkdir -p

# Output directory

OBJECTS_DIR   = objects/

# Files

DEPS    = $(FCLPATH)/MPU9150.h \
    $(FCLPATH)/I2Cdev.h \
    $(FCLPATH)/MPU9150_9Axis_MotionApps41.h \
    $(FCLPATH)/rs.hpp

OBJECTS = objects/flight.o \
    objects/MPU9150.o \
    objects/I2Cdev.o

MAKE_TARGET	= flight
DESTDIR		= Output/
TARGET		= Output/$(MAKE_TARGET)

# Build rules

$(TARGET): $(OBJECTS)
	@$(CHK_DIR_EXISTS) Output/ || $(MKDIR) Output/
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(LIBS)

clean:
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core

# Compile

$(OBJECTS_DIR)%.o : $(FCLPATH)/%.cpp $(DEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CXX) -c -o $@ $< $(CFLAGS) $(INCPATH)

$(OBJECTS_DIR)flight.o : flight.cpp $(DEPS)
	@$(CHK_DIR_EXISTS) objects/ || $(MKDIR) objects/
	$(CXX) -c -o $@ flight.cpp $(CFLAGS) $(INCPATH)

# Install

install_target: FORCE
	@$(CHK_DIR_EXISTS) $(INSTALL_ROOT)/usr/local/bin/ || $(MKDIR) $(INSTALL_ROOT)/usr/local/bin/
	-$(INSTALL_PROGRAM) "Output/$(MAKE_TARGET)" "$(INSTALL_ROOT)/usr/local/bin/$(MAKE_TARGET)"
	-$(STRIP) "$(INSTALL_ROOT)/usr/local/bin/$(MAKE_TARGET)"

uninstall_target:  FORCE
	-$(DEL_FILE) "$(INSTALL_ROOT)/usr/local/bin/$(MAKE_TARGET)"


install:  install_target  FORCE

uninstall: uninstall_target   FORCE

FORCE:

