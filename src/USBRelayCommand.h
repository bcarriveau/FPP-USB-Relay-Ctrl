#ifndef USBRELAYCOMMAND_H
#define USBRELAYCOMMAND_H

#include "fpp-pch.h"
#include "commands/Commands.h"
#include "Plugin.h"
#include <string>
#include <vector>
#include <map>
#include <thread>
#include <chrono>

class USBRelayCommand : public Command {
public:
    USBRelayCommand();
    virtual ~USBRelayCommand();
    virtual std::unique_ptr<Command::Result> run(const std::vector<std::string>& args) override;
    virtual Json::Value getDescription() override;

private:
    static std::map<std::string, bool> activeTimers;
    static std::map<std::string, int> deviceChannelCount;
    static std::map<std::string, std::string> deviceProtocol;

    static void initializeDevice(const std::string& device);
    static void turnOnRelay(const std::string& device, int channel);
    static void turnOffRelay(const std::string& device, int channel);
    static unsigned char getRelayBitmask(const std::string& device, int channel, bool turnOn);
    static void sendCH340Command(const std::string& device, int channel, bool turnOn);
};

class USBRelayPlugin : public FPPPlugin {
public:
    USBRelayPlugin();
    virtual ~USBRelayPlugin();
};

#endif // USBRELAYCOMMAND_H