#!/bin/bash

PWD=$(pwd)

export ANDROID_HOME=$PWD

$PWD/sync.sh

cd ftc_app-master

./gradlew installDebug

cd $PWD
