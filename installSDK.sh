#!/bin/bash

PWD=$(pwd)

mkdir -p ~/.android

echo "count=0" > ~/.android/repositories.cfg

echo "Installing android tools..."
$PWD/tools/bin/sdkmanager "tools"
echo "Installing build tools..."
$PWD/tools/bin/sdkmanager "build-tools;25.0.2"
$PWD/tools/bin/sdkmanager "build-tools;23.0.3"
echo "Installing sdk platforms (may be more than one)..."
$PWD/tools/bin/sdkmanager "platforms;android-25"
$PWD/tools/bin/sdkmanager "platforms;android-23"
$PWD/tools/bin/sdkmanager "platforms;android-19"
echo "Updating all packages..."
$PWD/tools/bin/sdkmanager --update
