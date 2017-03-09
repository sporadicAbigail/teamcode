#!/bin/bash

PWD=/home/ftc/pra_code

while inotifywait -r -e modify,create,delete $PWD/teamcode; do
    rsync -avz $PWD/teamcode/ $PWD/ftc_app-master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode --delete
done
