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

// Forward declaration of plugin_init
extern "C" void plugin_init();

// Define static members
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
                            LogWarn(VB_COMMAND, "Unknown USBRelay subType '%s' for device %s, skipping\n", 
                                    subType.c_str(), deviceName.c_str());
                            continue;
                        }
                        args.back().contentList.push_back(deviceName);
                        deviceProtocols[deviceName] = std::make_pair(protocol, channelCount);
                        LogDebug(VB_COMMAND, "Found device %s with protocol %s and %d channels\n", 
                                 deviceName.c_str(), protocol.c_str(), channelCount);
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

    // State argument
    args.emplace_back("State", "string", "Relay state", false);
    args.back().defaultValue = "";
    args.back().contentList = {"", "ON", "OFF", "ALL_OFF"};

    // Duration argument (in minutes)
    args.emplace_back("Duration", "int", "Duration in minutes to keep relay ON (0 for no auto-off)", true);
    args.back().min = 0;
    args.back().max = 999;
    args.back().defaultValue = "0";

    shutdownFlag = false;
}

void initializeICStation(const std::string& device) {
    int fd = open(device.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        LogErr(VB_COMMAND, "Failed to open %s for initialization\n", device.c_str());
        return;
    }
    unsigned char c_init = 0x50;
    unsigned char c_reply = 0x00;
    unsigned char c_open = 0x51;

    sleep(1);
    write(fd, &c_init, 1);
    usleep(500000);
    read(fd, &c_reply, 1);
    LogDebug(VB_COMMAND, "ICStation handshake response: 0x%02X\n", c_reply);
    write(fd, &c_open, 1);
    unsigned char wakeCmd = 0x00;
    write(fd, &wakeCmd, 1);
    usleep(100000);
    close(fd);

    USBRelayCommand::relayStates[device] = 0x00;
    LogDebug(VB_COMMAND, "Initialized ICStation device %s with handshake and wake-up\n", device.c_str());
}

unsigned char getRelayBitmask(const std::string& device, int channel, bool turnOn) {
    std::lock_guard<std::mutex> lock(USBRelayCommand::timerMutex);
    if (USBRelayCommand::relayStates.find(device) == USBRelayCommand::relayStates.end()) {
        USBRelayCommand::relayStates[device] = 0x00;
        LogDebug(VB_COMMAND, "Initialized relay state for %s to 0x00\n", device.c_str());
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
    LogDebug(VB_COMMAND, "Timer thread started for %s, channel %d, waiting %d minutes\n", device.c_str(), channel, durationMinutes);
    
    for (int i = 0; i < durationMinutes && USBRelayCommand::activeTimers[key] && !USBRelayCommand::shutdownFlag; i++) {
        std::this_thread::sleep_for(std::chrono::minutes(1));
    }
    
    std::lock_guard<std::mutex> lock(USBRelayCommand::timerMutex);
    if (USBRelayCommand::activeTimers[key] && !USBRelayCommand::shutdownFlag) {
        int fd = open(device.c_str(), O_WRONLY | O_NOCTTY);
        if (fd < 0) {
            LogErr(VB_COMMAND, "Failed to open %s for auto-off\n", device.c_str());
            USBRelayCommand::activeTimers.erase(key);
            return;
        }
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
            USBRelayCommand::activeTimers.erase(key);
            return;
        }
        ssize_t written = write(fd, cmd, len);
        close(fd);
        if (written == len) {
            LogDebug(VB_COMMAND, "Auto-turned off relay: %s, channel %d\n", device.c_str(), channel);
        } else {
            LogErr(VB_COMMAND, "Failed to auto-turn off relay, wrote %zd bytes\n", written);
        }
    }
    USBRelayCommand::activeTimers.erase(key);
}

std::unique_ptr<Command::Result> USBRelayCommand::run(const std::vector<std::string>& args) {
    if (args.size() < 3) return std::make_unique<Command::ErrorResult>("Device and State required, Channel optional");
    
    std::string device = args[0];
    std::string state = args[2];

    if (state == "ALL_OFF") {
        if (!args[1].empty()) {
            return std::make_unique<Command::ErrorResult>("Channel must be empty for ALL_OFF");
        }
        std::string resultStr = "Turned OFF all channels on devices: ";
        std::lock_guard<std::mutex> lock(USBRelayCommand::timerMutex);
        for (const auto& [dev, protoInfo] : USBRelayCommand::deviceProtocols) {
            std::string fullDevice = dev;
            if (fullDevice.find("/dev/") != 0) {
                fullDevice = "/dev/" + dev;
                LogDebug(VB_COMMAND, "Adjusted device path from %s to %s\n", dev.c_str(), fullDevice.c_str());
            }
            int fd = open(fullDevice.c_str(), O_WRONLY | O_NOCTTY);
            if (fd < 0) {
                LogErr(VB_COMMAND, "Failed to open %s\n", fullDevice.c_str());
                resultStr += dev + "(failed), ";
                continue;
            }
            std::string protocol = protoInfo.first;
            int channelCount = protoInfo.second;

            LogDebug(VB_COMMAND, "Turning off all %d channels on %s, protocol %s\n", 
                     channelCount, fullDevice.c_str(), protocol.c_str());

            if (protocol == "CH340") {
                for (int ch = 1; ch <= channelCount; ch++) {
                    unsigned char cmd[4] = {0xA0, (unsigned char)ch, 0x00, 0};
                    cmd[3] = cmd[0] + cmd[1] + cmd[2];
                    ssize_t written = write(fd, cmd, 4);
                    if (written != 4) {
                        LogErr(VB_COMMAND, "Failed to turn off %s channel %d, wrote %zd bytes\n", 
                               fullDevice.c_str(), ch, written);
                    }
                }
            } else if (protocol == "ICstation") {
                if (USBRelayCommand::relayStates.find(fullDevice) == USBRelayCommand::relayStates.end()) {
                    initializeICStation(fullDevice);
                }
                USBRelayCommand::relayStates[fullDevice] = 0x00;
                unsigned char cmd = USBRelayCommand::relayStates[fullDevice];
                ssize_t written = write(fd, &cmd, 1);
                if (written != 1) {
                    LogErr(VB_COMMAND, "Failed to turn off %s all channels, wrote %zd bytes\n", 
                           fullDevice.c_str(), written);
                }
            }
            close(fd);

            for (int ch = 1; ch <= channelCount; ch++) {
                std::string key = fullDevice + ":" + std::to_string(ch);
                USBRelayCommand::activeTimers[key] = false;
                auto it = USBRelayCommand::timerThreads.find(key);
                if (it != USBRelayCommand::timerThreads.end() && it->second.joinable()) {
                    it->second.join();
                    USBRelayCommand::timerThreads.erase(it);
                    LogDebug(VB_COMMAND, "Cancelled timer for %s channel %d\n", fullDevice.c_str(), ch);
                }
            }
            resultStr += dev + ", ";
        }
        if (resultStr == "Turned OFF all channels on devices: ") {
            return std::make_unique<Command::ErrorResult>("No devices configured for ALL_OFF");
        }
        resultStr = resultStr.substr(0, resultStr.length() - 2);
        return std::make_unique<Command::Result>(resultStr);
    }

    if (args[1].empty()) {
        return std::make_unique<Command::ErrorResult>("Channel required for ON or OFF state");
    }
    int channel;
    try {
        channel = std::stoi(args[1]);
    } catch (const std::exception& e) {
        LogErr(VB_COMMAND, "Invalid channel value: %s\n", args[1].c_str());
        return std::make_unique<Command::ErrorResult>("Invalid channel value: " + args[1]);
    }
    if (channel < 1 || channel > 8) return std::make_unique<Command::ErrorResult>("Channel must be 1-8");
    
    if (state != "ON" && state != "OFF") return std::make_unique<Command::ErrorResult>("State must be ON or OFF");

    int duration = 0;
    if (args.size() > 3 && !args[3].empty()) {
        try {
            duration = std::stoi(args[3]);
        } catch (const std::exception& e) {
            LogErr(VB_COMMAND, "Invalid duration value: %s\n", args[3].c_str());
            return std::make_unique<Command::ErrorResult>("Invalid duration value: " + args[3]);
        }
    }
    if (duration < 0) return std::make_unique<Command::ErrorResult>("Duration cannot be negative");

    std::string fullDevice = device;
    if (fullDevice.find("/dev/") != 0) {
        fullDevice = "/dev/" + device;
        LogDebug(VB_COMMAND, "Adjusted device path from %s to %s\n", device.c_str(), fullDevice.c_str());
    }
    std::string key = fullDevice + ":" + std::to_string(channel);

    auto it = USBRelayCommand::deviceProtocols.find(device);
    if (it == USBRelayCommand::deviceProtocols.end()) {
        LogErr(VB_COMMAND, "Device %s not configured in Others tab\n", device.c_str());
        return std::make_unique<Command::ErrorResult>("Device " + device + " not configured in Others tab");
    }
    std::string protocol = it->second.first;
    int channelCount = it->second.second;
    if (channel > channelCount) {
        LogErr(VB_COMMAND, "Channel %d exceeds configured count %d for %s\n", 
               channel, channelCount, device.c_str());
        return std::make_unique<Command::ErrorResult>("Channel " + args[1] + " exceeds configured count for " + device);
    }

    LogDebug(VB_COMMAND, "Opening device: %s, Channel: %d, State: %s, Protocol: %s, Duration: %d minutes\n", 
             fullDevice.c_str(), channel, state.c_str(), protocol.c_str(), duration);

    if (protocol == "ICstation" && USBRelayCommand::relayStates.find(fullDevice) == USBRelayCommand::relayStates.end()) {
        initializeICStation(fullDevice);
    }

    int fd = open(fullDevice.c_str(), O_WRONLY | O_NOCTTY);
    if (fd < 0) {
        LogErr(VB_COMMAND, "Failed to open %s\n", fullDevice.c_str());
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
        cmd[0] = getRelayBitmask(device, channel, state == "ON");
        len = 1;
    } else {
        close(fd);
        LogErr(VB_COMMAND, "Unsupported protocol %s for device %s\n", protocol.c_str(), device.c_str());
        return std::make_unique<Command::ErrorResult>("Unsupported protocol for device " + device);
    }

    LogDebug(VB_COMMAND, "Sending command to %s: [%02X %02X %02X %02X] (len=%zd)\n", 
             fullDevice.c_str(), cmd[0], cmd[1], cmd[2], cmd[3], len);

    ssize_t written = write(fd, cmd, len);
    close(fd);

    if (written != len) {
        LogErr(VB_COMMAND, "Write failed, wrote %zd bytes\n", written);
        return std::make_unique<Command::ErrorResult>("Failed to set relay channel " + args[1] + " to " + state);
    }

    LogDebug(VB_COMMAND, "Wrote to relay: %s, channel %d, state %s\n", 
             fullDevice.c_str(), channel, state.c_str());

    std::lock_guard<std::mutex> lock(USBRelayCommand::timerMutex);
    if (state == "ON" && duration > 0) {
        auto it = USBRelayCommand::timerThreads.find(key);
        if (it != USBRelayCommand::timerThreads.end() && it->second.joinable()) {
            USBRelayCommand::activeTimers[key] = false;
            it->second.join();
            USBRelayCommand::timerThreads.erase(it);
        }
        USBRelayCommand::activeTimers[key] = true;
        USBRelayCommand::timerThreads.emplace(key, std::thread([fullDevice, channel, protocol, duration, key]() { 
            turnOffAfterDelay(fullDevice, channel, protocol, duration, key); 
        }));
        LogDebug(VB_COMMAND, "Spawning timer thread for %s, channel %d, duration %d minutes\n", 
                 fullDevice.c_str(), channel, duration);
        return std::make_unique<Command::Result>("Relay channel " + args[1] + " turned ON for " + std::to_string(duration) + " minutes");
    } else if (state == "OFF") {
        USBRelayCommand::activeTimers[key] = false;
        auto it = USBRelayCommand::timerThreads.find(key);
        if (it != USBRelayCommand::timerThreads.end() && it->second.joinable()) {
            it->second.join();
            USBRelayCommand::timerThreads.erase(it);
        }
    }

    return std::make_unique<Command::Result>("Relay channel " + args[1] + " turned " + state);
}

void cleanupThreads() {
    std::lock_guard<std::mutex> lock(USBRelayCommand::timerMutex);
    USBRelayCommand::shutdownFlag = true;
    for (auto it = USBRelayCommand::timerThreads.begin(); it != USBRelayCommand::timerThreads.end(); ) {
        if (it->second.joinable()) {
            USBRelayCommand::activeTimers[it->first] = false;
            it->second.join();
            it = USBRelayCommand::timerThreads.erase(it);
        } else {
            ++it;
        }
    }
    USBRelayCommand::timerThreads.clear();
    USBRelayCommand::activeTimers.clear();
    LogDebug(VB_COMMAND, "All USB relay timer threads cleaned up\n");
}

// Define the plugin class
class USBRelayPlugin : public FPPPlugin {
public:
    USBRelayPlugin() : FPPPlugin("FPP-USB-Relay-Ctrl") {
        LogDebug(VB_PLUGIN, "USBRelayPlugin constructor called - plugin instantiated\n");
        plugin_init();
    }

    ~USBRelayPlugin() {
        LogDebug(VB_PLUGIN, "USBRelayPlugin destructor called\n");
    }
};

// Required by FPP to create the plugin instance
extern "C" FPPPlugin* createPlugin() {
    LogDebug(VB_PLUGIN, "Creating new USBRelayPlugin instance\n");
    return new USBRelayPlugin();
}

// Called during plugin initialization
extern "C" __attribute__((visibility("default"))) void plugin_init() {
    LogDebug(VB_PLUGIN, "Starting plugin_init for FPP-USB-Relay-Ctrl\n");
    CommandManager::INSTANCE.addCommand(new USBRelayCommand());
    LogDebug(VB_PLUGIN, "USB Relay On/Off command registered in plugin_init\n");
}

// Called during plugin cleanup
extern "C" __attribute__((visibility("default"))) void plugin_cleanup() {
    LogDebug(VB_PLUGIN, "Starting plugin_cleanup for FPP-USB-Relay-Ctrl\n");
    cleanupThreads();
    CommandManager::INSTANCE.removeCommand("USB Relay On/Off");
    LogDebug(VB_PLUGIN, "USB Relay On/Off command removed in plugin_cleanup\n");
}