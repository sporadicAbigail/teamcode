#!/bin/bash

PWD=$(pwd)

function pullLink {
  curl -L -# https://dl.google.com/android/repository/tools_r25.2.3-linux.zip > $PWD/android_tools.zip
  unzip $PWD/android_tools.zip
  rm $PWD/android_tools.zip
}

if [ ! -d $PWD/tools ]
  then pullLink  
  else echo -n "android-tools already exists, would you like to redownload it? (y/n): "
       read ui
       if [ $ui = "y" -o $ui = "Y" ]
         then rm -r $PWD/tools
              pullLink
       fi
fi
