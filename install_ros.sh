## install ros melodic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt update
sudo apt install -y ros-melodic-desktop

## setup ros environment
echo source /opt/ros/melodic/setup.bash >> ~/.bashrc
echo export ROS_MASTER_URI=http://localhost:11311 >> ~/.bashrc
echo export ROS_IP=localhost >> ~/.bashrc
echo export ROS_HOSTNAME=$ROS_IP >> ~/.bashrc

source ~/.bashrc
sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
## install catkin tools
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y python-catkin-tools
## install ros dependencies
sudo apt install -y ros-melodic-can-msgs 
## install ssh
sudo apt install -y openssh-server sshfs
