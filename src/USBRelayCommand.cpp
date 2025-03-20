#include "USBRelayCommand.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <map>
#include <sys/stat.h>

// Static member definitions
std::map<std::string, bool> USBRelayCommand::activeTimers;
std::map<std::string, int> USBRelayCommand::deviceChannelCount;
std::map<std::string, std::string> USBRelayCommand::deviceProtocol;
static std::map<std::string, unsigned char> relayStates;

// Initialize device based on protocol
void USBRelayCommand::initializeDevice(const std::string& device) {
    // Set permissions for the device
    if (chmod(device.c_str(), 0666) != 0) {
        LogErr(VB_COMMAND, "Failed to set permissions for %s: %s\n", device.c_str(), strerror(errno));
    }

    int fd = open(device.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        LogErr(VB_COMMAND, "Failed to open %s for initialization: %s\n", device.c_str(), strerror(errno));
        return;
    }

    std::string protocol = "ICstation"; // Default
    int channelCount = 8; // Default for ICstation
    std::string configFile = "/home/fpp/media/config/co-other.json";
    Json::Value config;
    if (FileExists(configFile) && LoadJsonFromFile(configFile, config)) {
        LogDebug(VB_COMMAND, "Parsing co-other.json for %s\n", device.c_str());
        if (config.isObject() && config.isMember("channelOutputs")) {
            const Json::Value& outputs = config["channelOutputs"];
            for (const auto& output : outputs) {
                std::string devName = output.get("device", "").asString();
                if (devName == device.substr(5)) {
                    std::string subType = output.get("subType", "").asString();
                    protocol = subType.empty() ? "ICstation" : subType;
                    channelCount = output.get("channelCount", protocol == "CH340" ? 1 : 8).asInt();
                    deviceChannelCount[device] = channelCount;
                    deviceProtocol[device] = protocol;
                    LogDebug(VB_COMMAND, "Detected %s with protocol %s, channels %d\n", devName.c_str(), protocol.c_str(), channelCount);
                    break;
                }
            }
        }
    }
    else {
        LogWarn(VB_COMMAND, "co-other.json not found or invalid at %s, using defaults\n", configFile.c_str());
    }

    if (protocol == "ICstation") {
        unsigned char c_init = 0x50;
        unsigned char c_reply = 0x00;
        unsigned char c_open = 0x51;

        sleep(1);
        write(fd, &c_init, 1);
        LogDebug(VB_COMMAND, "Sent init 0x50 to %s\n", device.c_str());
        usleep(500000);
        read(fd, &c_reply, 1);
        LogDebug(VB_COMMAND, "Handshake response: 0x%02X\n", c_reply);
        write(fd, &c_open, 1);
        LogDebug(VB_COMMAND, "Sent open 0x51 to %s\n", device.c_str());
        unsigned char wakeCmd = 0x00;
        write(fd, &wakeCmd, 1);
        LogDebug(VB_COMMAND, "Sent wake 0x00 to %s\n", device.c_str());
        usleep(100000);
    }
    else if (protocol == "CH340") {
        LogDebug(VB_COMMAND, "Initialized %s as CH340\n", device.c_str());
    }

    close(fd);
    relayStates[device] = 0x00;
    LogDebug(VB_COMMAND, "Initialized %s with state 0x00\n", device.c_str());
}

// Send CH340-specific command
void USBRelayCommand::sendCH340Command(const std::string& device, int channel, bool turnOn) {
    int fd = open(device.c_str(), O_WRONLY | O_NOCTTY);
    if (fd < 0) {
        LogErr(VB_COMMAND, "Failed to open %s for CH340 command: %s\n", device.c_str(), strerror(errno));
        return;
    }
    unsigned char cmd[4] = { 0xA0, (unsigned char)channel, (unsigned char)(turnOn ? 0x01 : 0x00), 0x00 };
    cmd[3] = cmd[0] + cmd[1] + cmd[2]; // Checksum
    if (write(fd, cmd, 4) != 4) {
        LogErr(VB_COMMAND, "Failed to send CH340 command to %s, channel %d: %s\n", device.c_str(), channel, strerror(errno));
    }
    else {
        LogDebug(VB_COMMAND, "Sent CH340 command to %s, channel %d, state %s\n", device.c_str(), channel, turnOn ? "ON" : "OFF");
        unsigned char bitmask = relayStates[device];
        if (turnOn) bitmask |= (1 << (channel - 1));
        else bitmask &= ~(1 << (channel - 1));
        relayStates[device] = bitmask;
    }
    close(fd);
}

USBRelayCommand::USBRelayCommand() : Command("USB Relay On/Off", "Controls USB relay with optional auto-off timer") {
    LogDebug(VB_COMMAND, "Constructing USBRelayCommand\n");

    // Device argument
    args.emplace_back("Device", "string", "USB Relay device from Others tab", false);
    std::string defaultDevice;

    // Populate devices from co-other.json
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
                    if (!deviceName.empty() && type == "USBRelay") {
                        args.back().contentList.push_back(deviceName);
                        LogDebug(VB_COMMAND, "Found device %s\n", deviceName.c_str());
                    }
                }
                if (!args.back().contentList.empty()) {
                    defaultDevice = args.back().contentList.front();
                    args.back().defaultValue = defaultDevice;
                    LogDebug(VB_COMMAND, "Set default device to %s\n", defaultDevice.c_str());
                }
            }
        }
    }
    else {
        LogWarn(VB_COMMAND, "co-other.json not found or invalid at %s, no USB relay devices available\n", configFile.c_str());
    }

    // Channel argument
    args.emplace_back("Channel", "int", "Relay channel (1-8)", false);
    args.back().min = 1;
    args.back().max = 8;
    args.back().defaultValue = "1";

    // State argument
    args.emplace_back("State", "string", "Relay state", false);
    args.back().contentList = { "ON", "OFF", "ALL_ON", "ALL_OFF" };
    args.back().defaultValue = "ON";

    // Duration argument
    args.emplace_back("Duration", "int", "Seconds to keep ON (0 for no auto-off)", true);
    args.back().min = 0;
    args.back().max = 999;
    args.back().defaultValue = "0";
}

USBRelayCommand::~USBRelayCommand() {
    // Clean up active timers
    activeTimers.clear();
}

std::unique_ptr<Command::Result> USBRelayCommand::run(const std::vector<std::string>& args) {
    if (args.size() < 3) {
        return std::make_unique<Command::ErrorResult>("Device, Channel, and State required");
    }

    std::string device = "/dev/" + args[0];
    int channel = std::stoi(args[1]);
    std::string state = args[2];
    int duration = (args.size() > 3) ? std::stoi(args[3]) : 0;

    // Validate inputs
    if (state != "ALL_ON" && state != "ALL_OFF") {
        if (channel < 1 || channel > 8) {
            return std::make_unique<Command::ErrorResult>("Channel must be 1-8 globally");
        }
        auto it = deviceChannelCount.find(device);
        int maxChannels = (it != deviceChannelCount.end()) ? it->second : 8;
        if (channel > maxChannels) {
            return std::make_unique<Command::ErrorResult>("Channel " + std::to_string(channel) + " exceeds device limit of " + std::to_string(maxChannels));
        }
    }

    if (state != "ON" && state != "OFF" && state != "ALL_ON" && state != "ALL_OFF") {
        return std::make_unique<Command::ErrorResult>("State must be ON, OFF, ALL_ON, or ALL_OFF");
    }

    // Initialize device if not already done
    if (relayStates.find(device) == relayStates.end()) {
        initializeDevice(device);
    }

    // Handle the relay state
    if (state == "ALL_ON" || state == "ALL_OFF") {
        std::string protocol = deviceProtocol[device];
        if (protocol == "ICstation") {
            unsigned char bitmask = (state == "ALL_ON") ? 0xFF : 0x00;
            int fd = open(device.c_str(), O_WRONLY | O_NOCTTY);
            if (fd >= 0) {
                if (write(fd, &bitmask, 1) != 1) {
                    LogErr(VB_COMMAND, "Failed to set %s to %s: %s\n", device.c_str(), state.c_str(), strerror(errno));
                }
                else {
                    LogDebug(VB_COMMAND, "Set %s to %s\n", device.c_str(), state.c_str());
                    relayStates[device] = bitmask;
                }
                close(fd);
            }
            else {
                LogErr(VB_COMMAND, "Failed to open %s: %s\n", device.c_str(), strerror(errno));
            }
        }
        else if (protocol == "CH340") {
            auto it = deviceChannelCount.find(device);
            int maxChannels = (it != deviceChannelCount.end()) ? it->second : 1;
            for (int ch = 1; ch <= maxChannels; ch++) {
                sendCH340Command(device, ch, state == "ALL_ON");
            }
        }
    }
    else {
        if (state == "ON") {
            turnOnRelay(device, channel);
        }
        else {
            turnOffRelay(device, channel);
        }
    }

    // Schedule auto-off if ON or ALL_ON with duration
    if ((state == "ON" || state == "ALL_ON") && duration > 0) {
        std::string key = device + ":" + std::to_string(channel);
        if (state == "ALL_ON") {
            key = device + ":ALL";
        }
        if (activeTimers[key]) {
            LogDebug(VB_COMMAND, "Timer already active for %s, channel %d, ignoring new timer\n", device.c_str(), channel);
            return std::make_unique<Command::Result>("Relay channel " + args[1] + " already ON with active timer");
        }
        activeTimers[key] = true;
        std::thread([key, device, channel, duration, state]() {
            std::this_thread::sleep_for(std::chrono::seconds(duration));
            if (activeTimers[key]) {
                if (state == "ALL_ON") {
                    std::string protocol = deviceProtocol[device];
                    if (protocol == "ICstation") {
                        unsigned char bitmask = 0x00;
                        int fd = open(device.c_str(), O_WRONLY | O_NOCTTY);
                        if (fd >= 0) {
                            if (write(fd, &bitmask, 1) != 1) {
                                LogErr(VB_COMMAND, "Failed to set %s to ALL_OFF: %s\n", device.c_str(), strerror(errno));
                            }
                            else {
                                LogDebug(VB_COMMAND, "Auto-turned off %s (ALL_OFF)\n", device.c_str());
                                relayStates[device] = bitmask;
                            }
                            close(fd);
                        }
                    }
                    else if (protocol == "CH340") {
                        auto it = deviceChannelCount.find(device);
                        int maxChannels = (it != deviceChannelCount.end()) ? it->second : 1;
                        for (int ch = 1; ch <= maxChannels; ch++) {
                            sendCH340Command(device, ch, false);
                        }
                    }
                }
                else {
                    turnOffRelay(device, channel);
                }
                activeTimers.erase(key);
            }
            }).detach();
            LogDebug(VB_COMMAND, "Scheduled auto-off for %s, channel %d in %d seconds\n", device.c_str(), channel, duration);
    }

    return std::make_unique<Command::Result>("Relay " + (state == "ALL_ON" || state == "ALL_OFF" ? "all channels" : "channel " + args[1]) + " turned " + state);
}

Json::Value USBRelayCommand::getDescription() {
    Json::Value desc;
    desc["name"] = "USB Relay On/Off";
    desc["description"] = "Controls USB relay with optional auto-off timer";
    desc["version"] = "1.1"; // Force UI refresh
    for (const auto& arg : args) {
        Json::Value argDesc;
        argDesc["name"] = arg.name;
        argDesc["type"] = arg.type;
        argDesc["description"] = arg.description;
        argDesc["optional"] = arg.optional;
        if (!arg.contentList.empty()) {
            for (const auto& content : arg.contentList) {
                argDesc["contents"].append(content);
            }
        }
        if (arg.min != arg.max) {
            argDesc["min"] = arg.min;
            argDesc["max"] = arg.max;
        }
        if (!arg.defaultValue.empty()) {
            argDesc["default"] = arg.defaultValue;
        }
        desc["args"].append(argDesc);
    }
    LogDebug(VB_COMMAND, "Generated description with %zu args\n", args.size());
    return desc;
}

unsigned char USBRelayCommand::getRelayBitmask(const std::string& device, int channel, bool turnOn) {
    if (relayStates.find(device) == relayStates.end()) {
        relayStates[device] = 0x00;
    }
    unsigned char bitmask = relayStates[device];
    if (turnOn) {
        bitmask |= (1 << (channel - 1));
    }
    else {
        bitmask &= ~(1 << (channel - 1));
    }
    relayStates[device] = bitmask;
    return bitmask;
}

void USBRelayCommand::turnOnRelay(const std::string& device, int channel) {
    std::string protocol = deviceProtocol[device];
    if (protocol == "CH340") {
        sendCH340Command(device, channel, true);
    }
    else {
        unsigned char bitmask = getRelayBitmask(device, channel, true);
        int fd = open(device.c_str(), O_WRONLY | O_NOCTTY);
        if (fd >= 0) {
            if (write(fd, &bitmask, 1) != 1) {
                LogErr(VB_COMMAND, "Failed to turn on %s, channel %d: %s\n", device.c_str(), channel, strerror(errno));
            }
            else {
                LogDebug(VB_COMMAND, "Turned on %s, channel %d\n", device.c_str(), channel);
            }
            close(fd);
        }
        else {
            LogErr(VB_COMMAND, "Failed to open %s: %s\n", device.c_str(), strerror(errno));
        }
    }
}

void USBRelayCommand::turnOffRelay(const std::string& device, int channel) {
    std::string protocol = deviceProtocol[device];
    if (protocol == "CH340") {
        sendCH340Command(device, channel, false);
    }
    else {
        unsigned char bitmask = getRelayBitmask(device, channel, false);
        int fd = open(device.c_str(), O_WRONLY | O_NOCTTY);
        if (fd >= 0) {
            if (write(fd, &bitmask, 1) != 1) {
                LogErr(VB_COMMAND, "Failed to turn off %s, channel %d: %s\n", device.c_str(), channel, strerror(errno));
            }
            else {
                LogDebug(VB_COMMAND, "Auto-turned off %s, channel %d\n", device.c_str(), channel);
            }
            close(fd);
        }
        else {
            LogErr(VB_COMMAND, "Failed to open %s: %s\n", device.c_str(), strerror(errno));
        }
    }
}

// Plugin registration
USBRelayPlugin::USBRelayPlugin() : FPPPlugin("FPP-USB-Relay-Ctrl") {
    LogDebug(VB_COMMAND, "Registering USB Relay On/Off command\n");
    CommandManager::INSTANCE.addCommand(new USBRelayCommand());
}

USBRelayPlugin::~USBRelayPlugin() {
    LogDebug(VB_COMMAND, "Unregistering USB Relay On/Off command\n");
    CommandManager::INSTANCE.removeCommand("USB Relay On/Off");
}

extern "C" FPPPlugin * createPlugin() {
    LogDebug(VB_COMMAND, "Creating FPP-USB-Relay-Ctrl plugin\n");
    return new USBRelayPlugin();
}