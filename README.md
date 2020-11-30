# NIA_AI_DATABASE
NIA_AI_DATABASE

## Logging Box

### Setting
#### Xavier Setting
* PC Name: xavier
* ID: xavier
* Password: .
* Change to **MAXN** mode first!
#### Clone repository
```
git clone https://github.com/Kyungpyo-Kim/NIA_AI_DATABASE.git --recursive
```
#### VScode (recommandation)
```bash
git clone https://github.com/JetsonHacksNano/installVSCode.git
cd installVSCode
./installVSCode.sh
code-oss
```
#### ROS
```bash
bash install_ros.sh
```
#### CUDA 10.0 (skip if you already have installed)
* [Download](https://drive.google.com/file/d/1VTWUsknY4sYBJ1e82FsOav4IrdCnAjq5/view?usp=sharing)
* Install
  ```bash
  sudo apt install libtbb2 libtbb-dev -y
  sudo dpkg -i cuda-repo-l4t-10-0-local-10.0.326_1.0-1_arm64.deb
  sudo apt-get update
  sudo apt-key add /var/cuda-repo-10-0-local-10.0.166/7fa2af80.pub
  sudo apt-get install cuda-toolkit-10.0

  echo "# Add 32-bit CUDA library & binary paths:" >> ~/.bashrc
  echo "export PATH=/usr/local/cuda-10.0/bin:$PATH" >> ~/.bashrc
  echo "export LD_LIBRARY_PATH=/usr/local/cuda-10.0/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
  source ~/.bashrc

  cd /usr/local/cuda/samples/1_Utilities/deviceQuery/
  sudo make
  ```
#### OpenCV
```bash
bash install_opencv.sh # takes ~30min
sudo ln -s /usr/include/opencv4 /usr/include/opencv
```
#### Pylon Camera Driver
* Download for AARCH64: https://drive.google.com/file/d/1SZe4kIC-2SsPWeN9AXjCdjawoqamj4dQ/view?usp=sharing
* Download for AMD64: https://drive.google.com/file/d/1f62sNouyTuJwT-zswJpjgEBdYGgtUBlg/view?usp=sharing
* Install
  ```bash
  tar -xf pylon_6.1.3.20159_aarch64_setup.tar.gz
  # tar -xvf pylon_6.1.1.19861_x86_64_setup.tar.gz
  sudo mkdir /opt/pylon
  sudo tar -C /opt/pylon -xzf ./pylon_6.1.3.20159_aarch64.tar.gz 
  # sudo tar -C /opt/pylon -xzf ./pylon_6.1.1.19861_x86_64.tar.gz
  sudo chmod 755 /opt/pylon
  sudo /opt/pylon/share/pylon/setup-usb.sh
  sudo /opt/pylon/bin/pylonviewer
  echo export PYLON_ROOT=/opt/pylon >> ~/.bashrc
  echo export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PYLON4/PYLON_ROOT/lib64 >> ~/.bashrc
  source ~/.bashrc
  ```
#### Kvaser
* Download: https://drive.google.com/file/d/1lKP4jiJdKNK78gwDvztmYq0zHCpSRHeX/view?usp=sharing
  ```bash
  tar -xvf linuxcan.tar.gz
  cd linuxcan
  make
  sudo make install
  reboot
  /usr/doc/canlib/examples/listChannels
  ## ... s/n XXXXX ... ## --> XXXXX is serial number
  ```
* Modify serial number in kvaser can launch file (NIA_AI_DATABASE/ros/src/app/logging_box/launch/kvaser_can_bridge_node.launch)
  ```xml
  <arg name="can_hardware_id" default="XXXXX" />
  ```
#### Ublox 
```bash
sudo chmod 666  /dev/ttyACM0
```
#### Setup SSD memory
```bash
## make mount point
mkdir -p ~/ssd

## Check disk
lsblk
# nvme0n1      259:11   0 931.5G  0 disk 

## Format
sudo mkfs.ext4 /dev/nvme0n1

## automatic mount on booting
sudo gedit /etc/fstab 
/dev/nvme0n1 /home/xavier/ssd ext4 defaults 0 2 ## 단일 컨트롤러, 단일 포트이기 때문에 nvme0n1 로 고정

## reboot and change permission
sudo reboot
df -h
sudo chmod 777 ~/ssd
```
### Build
* Build
  ```bash
  cd ros
  sudo sh -c 'echo "yaml https://raw.githubusercontent.com/basler/pylon-ros-camera/master/pylon_camera/rosdep/pylon_sdk.yaml" > /etc/ros/rosdep/sources.list.d/30-pylon_camera.list' && rosdep update && sudo rosdep install --from-paths . --ignore-src --rosdistro=$ROS_DISTRO -y
  catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && catkin build
  ## Building with release option takes about 5min.
  source devel/setup.bash
  mkdir -p ~/ssd/test
  roslaunch logging_box logging_box.launch logging_dir:=test
  ```
### Run
```bash
bash run.sh
```
#### Make "run.sh" executable
```bash
gsettings set org.gnome.nautilus.preferences executable-text-activation 'launch'
chmod +x run.sh
```

## Exporter

### UTC Time stamp
* Ublox gps 정보 중 **Navigation Position Velocity Time Solution(NAV-PVT (0x01 0x07))** 정보를 레퍼런스로 사용하여 계산
* 계산 방법
  - topic '/ublox_gps/navpvt' 의 첫번째 message 의 **1. gps time(NAV-PVT)** 와 **2. ros timestamp** 을 동일한 reference로 사용
  - 수신 받은 message의 ros timestamp와 **2. ros timestamp** 의 **3. 시간차이**를 계산
  - **1. gps time(NAV-PVT)** 정보에 **3. 시간차이**를 반영하여 gps weeks, gps time of week, utc 를 계산

### Run
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

### Run with sshfs
```bash
sshfs xavier@192.168.0.1:/home/xavier/ssd/202X-XX-XX-XX-XX-XX /my/database/folder
source /opt/ros/melodic/setup.bash
cd exporter
python exporter.py --bag_file=/my/database/folder
```
