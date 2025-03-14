#ifndef USBRELAYCOMMAND_H
#define USBRELAYCOMMAND_H

#include "commands/Commands.h"
#include <map>
#include <thread>
#include <atomic>
#include <mutex>

class USBRelayCommand : public Command {
public:
    USBRelayCommand();
    virtual std::unique_ptr<Command::Result> run(const std::vector<std::string>& args) override;

    // Static members
    static std::map<std::string, std::thread> timerThreads;
    static std::atomic<bool> shutdownFlag;
    static std::map<std::string, bool> activeTimers;
    static std::mutex timerMutex;
    static std::map<std::string, unsigned char> relayStates;
    static std::map<std::string, std::pair<std::string, int>> deviceProtocols;
};

void initializeICStation(const std::string& device);
void cleanupThreads();

#endif