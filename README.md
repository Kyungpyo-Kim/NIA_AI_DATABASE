# NIA_AI_DATABASE
NIA_AI_DATABASE

## Run
```bash
usage: exporter.py [-h] --bag_file BAG_FILE [--output_dir OUTPUT_DIR]
                   [--image_topic IMAGE_TOPIC] [--gps_topic GPS_TOPIC]
                   [--gps_time_topic GPS_TIME_TOPIC] [--can_topic CAN_TOPIC]

# example
source /opt/ros/melodic/setup.bash
cd exporter

python exporter.py --bag_file={bag_file_name}
# or
python exporter.py --bag_file={folder_of_bag_files}
```

## Run with sshfs
```bash
sshfs xavier@192.168.0.1:/home/xavier/ssd/202X-XX-XX-XX-XX-XX /my/database/folder
source /opt/ros/melodic/setup.bash
cd exporter
python exporter.py --bag_file=/my/database/folder
```