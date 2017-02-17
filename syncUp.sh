#!/bin/bash

PWD=$(pwd)

rm -r $PWD/ftc_app-master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode

cp -r $PWD/teamcode $PWD/ftc_app-master/TeamCode/src/main/java/org/firstinspires/ftc/
