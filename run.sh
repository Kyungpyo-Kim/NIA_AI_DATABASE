#!/bin/bash

## Set variables
logging_dir=$(date +'%Y_%m_%d_%H_%M_%S')
LOGGING_BOX_DIR=/home/${USER}/git/NIA_AI_DATABASE/ros
echo "Logging dir: /home/${USER}/ssd/${logging_dir}"
echo "Workspace dir: ${LOGGING_BOX_DIR}"

## create directory to save logging data
if [ ! -d /home/${USER}/ssd/${logging_dir} ]; then
  mkdir -p /home/${USER}/ssd/${logging_dir};
fi

## Create new session LoggingBox
byobu new-session -d -s LoggingBox

## Select default pane for roscore
byobu select-pane -t 0
byobu send-keys "source ${LOGGING_BOX_DIR}/devel/setup.bash" Enter
byobu send-keys "roscore" Enter
sleep 3

## Split pane 0 into two vertically stacked panes
byobu split-window -v

## Select the newly created pane for logging_box.launch
byobu select-pane -t 1
byobu send-keys "source ${LOGGING_BOX_DIR}/devel/setup.bash" Enter
byobu send-keys "roslaunch logging_box logging_box.launch" Enter

## Split pane 1 horizontally to create two side-by-side panes
byobu split-window -h

## Repeat the selection and splitting process with the top half
byobu select-pane -t 0
byobu split-window -h

## Select pane for rostopic
byobu select-pane -t 1
byobu send-keys "source ${LOGGING_BOX_DIR}/devel/setup.bash" Enter
byobu send-keys "rostopic hz /pylon_camera_node/image_raw /ublox_gps/fix /ublox_gps/navpvt /can_tx" Enter

## Select pane for rostopic
byobu select-pane -t 3
byobu send-keys "source ${LOGGING_BOX_DIR}/devel/setup.bash" Enter
byobu send-keys "rosbag record --split --size=5120 \
  -o /home/${USER}/ssd/${logging_dir}/logging \
  /pylon_camera_node/image_raw /ublox_gps/fix /ublox_gps/navpvt /can_tx" Enter

sleep 1

byobu
