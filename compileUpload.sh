#!/bin/bash

PWD=$(pwd)

export ANDROID_HOME=$PWD

cd ftc_app-master

./gradlew installDebug

cd $PWD
