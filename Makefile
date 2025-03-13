CC = g++
CFLAGS = -Wall -fPIC -I/opt/fpp/src -I/opt/fpp/src/commands
LDFLAGS = -shared
TARGET = FPP-USB-Relay-Ctrl.so
SRCS = src/USBRelayCommand.cpp
OBJS = $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(TARGET) $(OBJS)

install: $(TARGET)
	mkdir -p /home/fpp/media/plugins/FPP-USB-Relay-Ctrl
	cp $(TARGET) /home/fpp/media/plugins/FPP-USB-Relay-Ctrl/
	chown fpp:fpp /home/fpp/media/plugins/FPP-USB-Relay-Ctrl/$(TARGET)
	chmod 644 /home/fpp/media/plugins/FPP-USB-Relay-Ctrl/$(TARGET)

.PHONY: all clean install