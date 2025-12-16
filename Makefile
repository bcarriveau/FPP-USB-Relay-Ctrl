SRCDIR ?= /opt/fpp/src
include $(SRCDIR)/makefiles/common/setup.mk
include $(SRCDIR)/makefiles/platform/*.mk

TARGET = FPP-USB-Relay-Ctrl.so
SRCS = src/USBRelayCommand.cpp
OBJECTS = $(SRCS:.cpp=.o)

all: $(TARGET)

debug: all

CFLAGS += -I. -I$(SRCDIR) -I$(SRCDIR)/commands
CXXFLAGS += $(CFLAGS)
LIBS += -L$(SRCDIR) -lfpp -ljsoncpp

%.o: %.cpp Makefile
	$(CCACHE) $(CXX) $(CXXFLAGS) $(CXXFLAGS_$@) -c $< -o $@

$(TARGET): $(OBJECTS) $(SRCDIR)/libfpp.$(SHLIB_EXT)
	$(CCACHE) $(CXX) -shared $(CXXFLAGS) $(OBJECTS) $(LIBS) $(LDFLAGS) -o $@

install: $(TARGET)
	mkdir -p /home/fpp/media/plugins/FPP-USB-Relay-Ctrl
	cp $(TARGET) /home/fpp/media/plugins/FPP-USB-Relay-Ctrl/
	chown fpp:fpp /home/fpp/media/plugins/FPP-USB-Relay-Ctrl/$(TARGET)
	chmod 644 /home/fpp/media/plugins/FPP-USB-Relay-Ctrl/$(TARGET)

clean:
	rm -f $(TARGET) $(OBJECTS)