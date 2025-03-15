TARGET = FPP-USB-Relay-Ctrl.so
SRCS = src/USBRelayCommand.cpp

all: $(TARGET)

$(TARGET): $(SRCS)
    g++ -shared -fPIC -I/opt/fpp/src -I/opt/fpp/src/commands -o $(TARGET) $(SRCS)

install: $(TARGET)
    sudo mkdir -p /home/fpp/media/plugins/FPP-USB-Relay-Ctrl
    sudo cp $(TARGET) /home/fpp/media/plugins/FPP-USB-Relay-Ctrl/
    sudo chown fpp:fpp /home/fpp/media/plugins/FPP-USB-Relay-Ctrl/$(TARGET)
    sudo chmod 644 /home/fpp/media/plugins/FPP-USB-Relay-Ctrl/$(TARGET)

clean:
    rm -f $(TARGET) src/USBRelayCommand.o