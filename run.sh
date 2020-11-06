#!/bin/bash

logging_dir=$(date +'%Y_%m_%d_%H_%M_%S')
LOGGING_BOX_DIR=/home/${USER}/git/logging_box/ros
echo "Logging dir: /home/${USER}/ssd/${logging_dir}"
echo "Workspace dir: ${LOGGING_BOX_DIR}"


if [ ! -d /home/${USER}/ssd/${logging_dir} ]; then
  mkdir -p /home/${USER}/ssd/${logging_dir};
fi

source ${LOGGING_BOX_DIR}/devel/setup.bash
gnome-terminal --disable-factory --working-directory=${LOGGING_BOX_DIR} \
  -- roslaunch logging_box logging_box.launch&

sleep 3

echo ${logging_dir}
source ${LOGGING_BOX_DIR}/devel/setup.bash
gnome-terminal --disable-factory --working-directory=${LOGGING_BOX_DIR} \
  -- rostopic hz \
  /pylon_camera_node/image_raw /ublox_gps/fix /ublox_gps/navpvt /can_tx&

sleep 3

echo ${logging_dir}
source ${LOGGING_BOX_DIR}/devel/setup.bash
gnome-terminal --disable-factory --working-directory=${LOGGING_BOX_DIR} \
  -- rosbag record --split --size=5120 \
  -o /home/${USER}/ssd/${logging_dir}/logging \
  /pylon_camera_node/image_raw /ublox_gps/fix /ublox_gps/navpvt /can_tx&

