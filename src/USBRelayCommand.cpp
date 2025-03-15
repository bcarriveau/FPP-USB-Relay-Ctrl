#include "fpp-pch.h"
#include "USBRelayCommand.h"
#include "commands/Commands.h"
#include "Plugin.h"
#include <thread>
#include <chrono>
#include <mutex>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/select.h>

// Forward declaration of plugin_init
extern "C" void plugin_init();

// Static member definitions
std::map<std::string, std::thread> USBRelayCommand::timerThreads;
std::atomic<bool> USBRelayCommand::shutdownFlag{false};
std::map<std::string, bool> USBRelayCommand::activeTimers;
std::mutex USBRelayCommand::timerMutex;
std::map<std::string, unsigned char> USBRelayCommand::relayStates;
std::map<std::string, std::pair<std::string, int>> USBRelayCommand::deviceProtocols;

USBRelayCommand::USBRelayCommand() : Command("USB Relay On/Off", "Turns a USB relay channel ON or OFF, optionally for a specified duration in minutes") {
    // Device argument
    args.emplace_back("Device", "string", "USB Relay device from Others tab", false);
    std::string defaultDevice;

    // Populate devices and protocols from co-other.json
    std::string configFile = "/home/fpp/media/config/co-other.json";
    Json::Value config;
    if (FileExists(configFile) && LoadJsonFromFile(configFile, config)) {
        LogDebug(VB_COMMAND, "Parsing co-other.json for USB relay devices\n");
        if (config.isObject() && config.isMember("channelOutputs")) {
            const Json::Value& outputs = config["channelOutputs"];
            if (outputs.isArray()) {
                LogDebug(VB_COMMAND, "Found %u channel outputs in co-other.json\n", outputs.size());
                for (const auto& output : outputs) {
                    std::string deviceName = output.get("device", "").asString();
                    std::string type = output.get("type", "").asString();
                    std::string subType = output.get("subType", "").asString();
                    int channelCount = output.get("channelCount", 8).asInt();
                    if (!deviceName.empty() && type == "USBRelay") {
                        std::string protocol;
                        if (subType == "CH340") {
                            protocol = "CH340";
                        } else if (subType == "ICStation") {
                            protocol = "ICstation";
                        } else {
                            LogWarn(VB_COMMAND, "Unknown USBRelay subType '%s' for device %s, skipping\n", subType.c_str(), deviceName.c_str());
                            continue;
                        }
                        args.back().contentList.push_back(deviceName);
                        USBRelayCommand::deviceProtocols[deviceName] = std::make_pair(protocol, channelCount);
                        LogDebug(VB_COMMAND, "Found device %s with protocol %s and %d channels\n", deviceName.c_str(), protocol.c_str(), channelCount);
                    }
                }
                if (!args.back().contentList.empty()) {
                    defaultDevice = args.back().contentList.front();
                    args.back().defaultValue = defaultDevice;
                    LogDebug(VB_COMMAND, "Set default device to %s\n", defaultDevice.c_str());
                }
            }
        }
    } else {
        LogWarn(VB_COMMAND, "co-other.json not found or invalid at %s, no USB relay devices available\n", configFile.c_str());
    }

    // Channel argument
    args.emplace_back("Channel", "int", "Relay channel (1-8). Leave empty for ALL_OFF.", true);
    args.back().min = 1;
    args.back().max = 8;
    args.back().defaultValue = "";

    // State argument - Removed blank option, no default selection
    args.emplace_back("State", "string", "Relay state", false);
    args.back().contentList = {"ON", "OFF", "ALL_OFF"}; // No "" option
    args.back().defaultValue = ""; // Empty default - nothing selected

    // Duration argument (in minutes)
    args.emplace_back("Duration", "int", "Duration in minutes to keep relay ON (0 for no auto-off)", true);
    args.back().min = 0;
    args.back().max = 999;
    args.back().defaultValue = "0";

    USBRelayCommand::shutdownFlag = false;
}

void initializeICStation(const std::string& device) {
    int fd = open(device.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        LogErr(VB_COMMAND, "Failed to open %s for ICStation initialization: %s\n", device.c_str(), strerror(errno));
        return;
    }

    unsigned char c_init = 0x50;
    unsigned char c_reply = 0x00;
    unsigned char c_open = 0x51;

    sleep(1);
    if (write(fd, &c_init, 1) != 1) {
        LogErr(VB_COMMAND, "Failed to write init command to %s: %s\n", device.c_str(), strerror(errno));
        close(fd);
        return;
    }
    usleep(500000); // 500ms delay

    // Add timeout to prevent hangs
    fd_set readfds;
    struct timeval timeout;
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);
    timeout.tv_sec = 1; // 1-second timeout
    timeout.tv_usec = 0;

    int ret = select(fd + 1, &readfds, nullptr, nullptr, &timeout);
    if (ret > 0 && FD_ISSET(fd, &readfds)) {
        if (read(fd, &c_reply, 1) == 1) {
            LogDebug(VB_COMMAND, "ICStation handshake response from %s: 0x%02X\n", device.c_str(), c_reply);
        } else {
            LogWarn(VB_COMMAND, "Failed to read full handshake response from %s: %s\n", device.c_str(), strerror(errno));
        }
    } else if (ret == 0) {
        LogWarn(VB_COMMAND, "Timeout waiting for ICStation handshake from %s\n", device.c_str());
    } else {
        LogErr(VB_COMMAND, "Select error on %s: %s\n", device.c_str(), strerror(errno));
    }

    if (write(fd, &c_open, 1) != 1 || write(fd, "\x00", 1) != 1) {
        LogErr(VB_COMMAND, "Failed to write open/wake command to %s: %s\n", device.c_str(), strerror(errno));
    }
    usleep(100000); // 100ms delay
    close(fd);

    USBRelayCommand::relayStates[device] = 0x00;
    LogDebug(VB_COMMAND, "Initialized ICStation device %s\n", device.c_str());
}

unsigned char getRelayBitmask(const std::string& device, int channel, bool turnOn) {
    std::lock_guard<std::mutex> lock(USBRelayCommand::timerMutex);
    if (USBRelayCommand::relayStates.find(device) == USBRelayCommand::relayStates.end()) {
        USBRelayCommand::relayStates[device] = 0x00;
    }
    unsigned char bitmask = USBRelayCommand::relayStates[device];
    if (turnOn) {
        bitmask |= (1 << (channel - 1));
    } else {
        bitmask &= ~(1 << (channel - 1));
    }
    USBRelayCommand::relayStates[device] = bitmask;
    return bitmask;
}

void turnOffAfterDelay(const std::string& device, int channel, const std::string& protocol, int durationMinutes, const std::string& key) {
    LogDebug(VB_COMMAND, "Timer started for %s, channel %d, duration %d minutes\n", device.c_str(), channel, durationMinutes);
    for (int i = 0; i < durationMinutes && USBRelayCommand::activeTimers[key] && !USBRelayCommand::shutdownFlag; i++) {
        std::this_thread::sleep_for(std::chrono::minutes(1));
    }

    std::lock_guard<std::mutex> lock(USBRelayCommand::timerMutex);
    if (USBRelayCommand::activeTimers[key] && !USBRelayCommand::shutdownFlag) {
        int fd = open(device.c_str(), O_WRONLY | O_NOCTTY);
        if (fd < 0) {
            LogErr(VB_COMMAND, "Failed to open %s for auto-off: %s\n", device.c_str(), strerror(errno));
        } else {
            unsigned char cmd[4] = {0};
            ssize_t len;
            if (protocol == "CH340") {
                cmd[0] = 0xA0;
                cmd[1] = (unsigned char)channel;
                cmd[2] = 0x00; // OFF
                cmd[3] = cmd[0] + cmd[1] + cmd[2];
                len = 4;
            } else if (protocol == "ICstation") {
                cmd[0] = getRelayBitmask(device, channel, false);
                len = 1;
            } else {
                close(fd);
                return;
            }
            if (write(fd, cmd, len) == len) {
                LogDebug(VB_COMMAND, "Auto-turned off %s, channel %d\n", device.c_str(), channel);
            } else {
                LogErr(VB_COMMAND, "Failed to auto-turn off %s, channel %d: %s\n", device.c_str(), channel, strerror(errno));
            }
            close(fd);
        }
    }
    USBRelayCommand::activeTimers.erase(key);
}

std::unique_ptr<Command::Result> USBRelayCommand::run(const std::vector<std::string>& args) {
    if (args.size() < 3) {
        return std::make_unique<Command::ErrorResult>("Device and State required, Channel optional");
    }

    std::string device = args[0];
    std::string state = args[2];
    std::string fullDevice = (device.find("/dev/") == 0) ? device : "/dev/" + device;

    if (state == "ALL_OFF") {
        // Ignore channel if specified with ALL_OFF - simulates auto-clear by not enforcing empty
        if (!args[1].empty()) {
            LogWarn(VB_COMMAND, "Channel specified (%s) with ALL_OFF, ignoring channel\n", args[1].c_str());
        }
        std::string resultStr = "Turned OFF all channels on: ";
        std::lock_guard<std::mutex> lock(USBRelayCommand::timerMutex);
        for (const auto& [dev, protoInfo] : USBRelayCommand::deviceProtocols) {
            std::string devPath = (dev.find("/dev/") == 0) ? dev : "/dev/" + dev;
            int fd = open(devPath.c_str(), O_WRONLY | O_NOCTTY);
            if (fd < 0) {
                LogErr(VB_COMMAND, "Failed to open %s for ALL_OFF: %s\n", devPath.c_str(), strerror(errno));
                resultStr += dev + "(failed), ";
                continue;
            }
            std::string protocol = protoInfo.first;
            int channelCount = protoInfo.second;

            if (protocol == "CH340") {
                for (int ch = 1; ch <= channelCount; ch++) {
                    unsigned char cmd[4] = {0xA0, (unsigned char)ch, 0x00, 0};
                    cmd[3] = cmd[0] + cmd[1] + cmd[2];
                    if (write(fd, cmd, 4) != 4) {
                        LogErr(VB_COMMAND, "Failed to turn off channel %d on %s: %s\n", ch, devPath.c_str(), strerror(errno));
                    }
                }
            } else if (protocol == "ICstation") {
                if (USBRelayCommand::relayStates.find(devPath) == USBRelayCommand::relayStates.end()) {
                    initializeICStation(devPath);
                }
                USBRelayCommand::relayStates[devPath] = 0x00;
                if (write(fd, &USBRelayCommand::relayStates[devPath], 1) != 1) {
                    LogErr(VB_COMMAND, "Failed to turn off all channels on %s: %s\n", devPath.c_str(), strerror(errno));
                }
            }
            close(fd);
            resultStr += dev + ", ";
        }
        if (resultStr == "Turned OFF all channels on: ") {
            return std::make_unique<Command::ErrorResult>("No devices configured for ALL_OFF");
        }
        return std::make_unique<Command::Result>(resultStr.substr(0, resultStr.length() - 2));
    }

    if (args[1].empty()) {
        return std::make_unique<Command::ErrorResult>("Channel required for ON or OFF");
    }
    int channel;
    try {
        channel = std::stoi(args[1]);
    } catch (const std::exception& e) {
        LogErr(VB_COMMAND, "Invalid channel value: %s\n", args[1].c_str());
        return std::make_unique<Command::ErrorResult>("Invalid channel value: " + args[1]);
    }
    if (channel < 1 || channel > 8) {
        return std::make_unique<Command::ErrorResult>("Channel must be 1-8");
    }
    if (state != "ON" && state != "OFF") {
        return std::make_unique<Command::ErrorResult>("State must be ON or OFF");
    }

    int duration = (args.size() > 3 && !args[3].empty()) ? std::stoi(args[3]) : 0;
    if (duration < 0) {
        return std::make_unique<Command::ErrorResult>("Duration cannot be negative");
    }

    auto it = USBRelayCommand::deviceProtocols.find(device);
    if (it == USBRelayCommand::deviceProtocols.end()) {
        return std::make_unique<Command::ErrorResult>("Device " + device + " not configured");
    }
    std::string protocol = it->second.first;
    int channelCount = it->second.second;
    if (channel > channelCount) {
        return std::make_unique<Command::ErrorResult>("Channel " + args[1] + " exceeds configured count for " + device);
    }

    LogDebug(VB_COMMAND, "Processing device %s, Channel %d, State %s, Protocol %s, Duration %d minutes\n", 
             fullDevice.c_str(), channel, state.c_str(), protocol.c_str(), duration);

    if (protocol == "ICstation" && USBRelayCommand::relayStates.find(fullDevice) == USBRelayCommand::relayStates.end()) {
        initializeICStation(fullDevice);
    }

    int fd = open(fullDevice.c_str(), O_WRONLY | O_NOCTTY);
    if (fd < 0) {
        LogErr(VB_COMMAND, "Failed to open %s: %s\n", fullDevice.c_str(), strerror(errno));
        return std::make_unique<Command::ErrorResult>("Failed to open device " + device);
    }

    unsigned char cmd[4] = {0};
    ssize_t len;
    if (protocol == "CH340") {
        cmd[0] = 0xA0;
        cmd[1] = (unsigned char)channel;
        cmd[2] = (state == "ON") ? 0x01 : 0x00;
        cmd[3] = cmd[0] + cmd[1] + cmd[2];
        len = 4;
    } else if (protocol == "ICstation") {
        cmd[0] = getRelayBitmask(fullDevice, channel, state == "ON");
        len = 1;
    } else {
        close(fd);
        return std::make_unique<Command::ErrorResult>("Unsupported protocol for " + device);
    }

    if (write(fd, cmd, len) != len) {
        LogErr(VB_COMMAND, "Failed to write to %s: %s\n", fullDevice.c_str(), strerror(errno));
        close(fd);
        return std::make_unique<Command::ErrorResult>("Failed to set relay channel " + args[1] + " to " + state);
    }
    LogDebug(VB_COMMAND, "Successfully wrote %zd bytes to %s\n", len, fullDevice.c_str());
    close(fd);

    std::lock_guard<std::mutex> lock(USBRelayCommand::timerMutex);
    std::string key = fullDevice + ":" + std::to_string(channel);
    if (state == "ON" && duration > 0) {
        if (USBRelayCommand::timerThreads.count(key) && USBRelayCommand::timerThreads[key].joinable()) {
            USBRelayCommand::activeTimers[key] = false;
            USBRelayCommand::timerThreads[key].join();
            USBRelayCommand::timerThreads.erase(key);
        }
        USBRelayCommand::activeTimers[key] = true;
        USBRelayCommand::timerThreads[key] = std::thread([fullDevice, channel, protocol, duration, key]() {
            turnOffAfterDelay(fullDevice, channel, protocol, duration, key);
        });
        return std::make_unique<Command::Result>("Relay channel " + args[1] + " turned ON for " + std::to_string(duration) + " minutes");
    } else if (state == "OFF") {
        USBRelayCommand::activeTimers[key] = false;
        if (USBRelayCommand::timerThreads.count(key) && USBRelayCommand::timerThreads[key].joinable()) {
            USBRelayCommand::timerThreads[key].join();
            USBRelayCommand::timerThreads.erase(key);
        }
    }

    return std::make_unique<Command::Result>("Relay channel " + args[1] + " turned " + state);
}

void cleanupThreads() {
    std::lock_guard<std::mutex> lock(USBRelayCommand::timerMutex);
    USBRelayCommand::shutdownFlag = true;
    for (auto& [key, thread] : USBRelayCommand::timerThreads) {
        if (thread.joinable()) {
            USBRelayCommand::activeTimers[key] = false;
            thread.join();
        }
    }
    USBRelayCommand::timerThreads.clear();
    USBRelayCommand::activeTimers.clear();
    LogDebug(VB_COMMAND, "All USB relay timer threads cleaned up\n");
}

class USBRelayPlugin : public FPPPlugin {
public:
    USBRelayPlugin() : FPPPlugin("FPP-USB-Relay-Ctrl") {
        plugin_init();
    }
    ~USBRelayPlugin() {}
};

extern "C" FPPPlugin* createPlugin() {
    return new USBRelayPlugin();
}

extern "C" __attribute__((visibility("default"))) void plugin_init() {
    CommandManager::INSTANCE.addCommand(new USBRelayCommand());
    LogDebug(VB_PLUGIN, "Initialized FPP-USB-Relay-Ctrl plugin\n");
}

extern "C" __attribute__((visibility("default"))) void plugin_cleanup() {
    cleanupThreads();
    CommandManager::INSTANCE.removeCommand("USB Relay On/Off");
    LogDebug(VB_PLUGIN, "Cleaned up FPP-USB-Relay-Ctrl plugin\n");
}