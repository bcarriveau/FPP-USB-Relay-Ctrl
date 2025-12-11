WIP

# FPP-USB-Relay-Ctrl

A plugin for Falcon Player (FPP) to control USB relays with on/off commands.

## Installation for End Users

This plugin can be installed directly via the FPP Plugin Manager:

1. Open the FPP web interface.
2. Go to "Content Setup" > "Plugins".
3. In the "Install Plugin" section, enter the repository URL:`https://github.com/bcarriveau/FPP-USB-Relay-Ctrl/pluginInfo.json`
4. Click "Install".
5. Once installed, the "USB Relay On/Off" command will be available in "Command Presets" or the "Run FPP Command" button on the Status page.

### Usage
- Use the "USB Relay On/Off" command to control USB relays.
- Arguments:
- **Device**: Select the USB relay device (configured in the "Others" tab).
- **Channel**: Relay channel (1-8), leave empty for "ALL_OFF".
- **State**: Select "ON", "OFF", or "ALL_OFF" (defaults to blank, requiring selection).
- **Duration**: Duration in minutes to keep the relay ON (0 for no auto-off).

## Development
For developers who want to modify the plugin:
- Clone the repository:`https://github.com/bcarriveau/FPP-USB-Relay-Ctrl.git`
- Build the plugin - `cd opt/fpp/plugins/FPP-USB-Relay-Ctrl/` - `sudo make clean && sudo make`

- Copy the built `FPP-USB-Relay-Ctrl.so` to `/home/fpp/media/plugins/FPP-USB-Relay-Ctrl/` 
- `sudo cp FPP-USB-Relay-Ctrl.so /home/fpp/media/plugins/FPP-USB-Relay-Ctrl/`
- Restart FPP: `sudo systemctl restart fppd`


