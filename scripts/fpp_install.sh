#!/bin/bash

# Define paths
PLUGIN_NAME="FPP-USB-Relay-Ctrl"
PLUGIN_HOME="/home/fpp/media/plugins/$PLUGIN_NAME"
BUILD_DIR="/opt/fpp/plugins/$PLUGIN_NAME"

echo "Running install script for $PLUGIN_NAME"

# Step 1: Create temporary build directory
echo "Creating build directory $BUILD_DIR..."
sudo mkdir -p "$BUILD_DIR/src"

# Step 2: Copy source files to build directory, preserving the src/ structure
echo "Copying source files to $BUILD_DIR..."
sudo cp -r "$PLUGIN_HOME/src/"* "$BUILD_DIR/src/"
sudo cp "$PLUGIN_HOME/Makefile" "$BUILD_DIR/"

# Step 3: Build the plugin
echo "Building $PLUGIN_NAME.so in $BUILD_DIR..."
cd "$BUILD_DIR" || {
    echo "Error: Could not change to $BUILD_DIR"
    exit 1
}
sudo make clean
if ! sudo make; then
    echo "Error: Failed to build $PLUGIN_NAME.so"
    exit 1
fi

# Step 4: Move the .so file to the plugin home directory
echo "Moving $PLUGIN_NAME.so to $PLUGIN_HOME..."
sudo cp "$BUILD_DIR/$PLUGIN_NAME.so" "$PLUGIN_HOME/"
sudo chown fpp:fpp "$PLUGIN_HOME/$PLUGIN_NAME.so"
sudo chmod 644 "$PLUGIN_HOME/$PLUGIN_NAME.so"

# Step 5: Clean up the build directory
echo "Cleaning up $BUILD_DIR..."
sudo rm -rf "$BUILD_DIR"

echo "Install complete"
echo "Note: Restart fppd to apply plugin updates"