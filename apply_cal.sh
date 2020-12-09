#!/bin/bash
  
cd /tmp
tar -xzvf calibrationdata.tar.gz
mv ost.txt ost.ini
rosrun camera_calibration_parsers convert ost.ini camera.yaml

config_folder=~/git/NIA_AI_DATABASE/ros/src/app/logging_box/config

cp camera.yaml /home/${USER}/ssd/calibration_$(date +'%Y_%m_%d_%H_%M').yaml
cp camera.yaml ${config_folder}

