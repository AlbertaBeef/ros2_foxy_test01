#!/bin/bash
#
# Copyright 2022 Xilinx, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

set CWD=$(pwd)

# Installing some useful utilities
sudo apt update
sudo apt-get install -y net-tools mlocate libgoogle-glog-dev libpcap0.8-dev

# Install ROS2
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
echo "source /opt/ros/foxy/setup.bash" >> ${HOME}/.bashrc

# Install ROS2 packages we use
sudo apt install -y ros-foxy-desktop python3-colcon-common-extensions ros-foxy-diagnostic-updater ros-foxy-camera-info-manager ros-foxy-velodyne
source /opt/ros/foxy/setup.bash

# Install cv_camera ROS2 package
mkdir ${HOME}/cv_camera_ws
cd ${HOME}/cv_camera_ws 
	git clone https://github.com/Kapernikov/cv_camera src
	colcon build
	echo "source ${HOME}/cv_camera_ws/install/setup.bash" >> ${HOME}/.bashrc
cd ${CWD}


sudo bash -c 'cat << EOF > /etc/systemd/network/20-wired.network
[Match]
Name=eth0

[Network]
DHCP=yes

[Route]
Destination=192.168.1.201
EOF'
sudo systemctl restart systemd-networkd.service

# Install the Xilinx KV260 Apps
sudo snap install xlnx-config --edge --classic
yes Y | sudo xlnx-config.sysinit
sudo snap install xlnx-nlp-smartvision
xlnx-config -q
sudo xlnx-config --install nlp-smartvision
sudo xlnx-config --xmutil loadapp nlp-smartvision
sudo bash -c 'cat << EOF > /usr/local/bin/load_firmware.sh
#!/bin/bash

xlnx-config --xmutil loadapp nlp-smartvision
EOF'
sudo chmod 744 /usr/local/bin/load_firmware.sh
sudo bash -c 'cat <<EOF > /etc/systemd/system/load_firmware.service
[Unit]
After=network.service

[Service]
ExecStart=/usr/local/bin/load_firmware.sh

[Install]
WantedBy=default.target
EOF'
sudo systemctl start load_firmware
sudo systemctl enable load_firmware

sudo snap install xlnx-vai-lib-samples

# Download Vitis AI models that will be used
sudo mkdir -p /usr/share/vitis_ai_library/models
mkdir -p ${HOME}/Downloads
wget https://www.xilinx.com/bin/public/openDownload?filename=yolov3_bdd-DPUCZDX8G_ISA0_B3136_MAX_BG2-1.3.1-r241.tar.gz -O ${HOME}/Downloads/yolov3_bdd-DPUCZDX8G_ISA0_B3136_MAX_BG2-1.3.1-r241.tar.gz
sudo tar -xvf ${HOME}/Downloads/yolov3_bdd-DPUCZDX8G_ISA0_B3136_MAX_BG2-1.3.1-r241.tar.gz -C /usr/share/vitis_ai_library/models

wget  https://www.xilinx.com/bin/public/openDownload?filename=yolov3_adas_pruned_0_9-DPUCZDX8G_ISA0_B3136_MAX_BG2-1.3.1-r241.tar.gz -O ${HOME}/Downloads/yolov3_adas_pruned_0_9-DPUCZDX8G_ISA0_B3136_MAX_BG2-1.3.1-r241.tar.gz
sudo tar -xvf ${HOME}/Downloads/yolov3_adas_pruned_0_9-DPUCZDX8G_ISA0_B3136_MAX_BG2-1.3.1-r241.tar.gz -C /usr/share/vitis_ai_library/models

wget https://www.xilinx.com/bin/public/openDownload?filename=pointpillars_kitti_12000_0_pt-DPUCZDX8G_ISA0_B3136_MAX_BG2-1.3.1-r241.tar.gz -O ${HOME}/Downloads/pointpillars_kitti_12000_0_pt-DPUCZDX8G_ISA0_B3136_MAX_BG2-1.3.1-r241.tar.gz
sudo tar -xvf ${HOME}/Downloads/pointpillars_kitti_12000_0_pt-DPUCZDX8G_ISA0_B3136_MAX_BG2-1.3.1-r241.tar.gz -C /usr/share/vitis_ai_library/models

wget https://www.xilinx.com/bin/public/openDownload?filename=pointpillars_kitti_12000_1_pt-DPUCZDX8G_ISA0_B3136_MAX_BG2-1.3.1-r241.tar.gz -O ${HOME}/Downloads/pointpillars_kitti_12000_1_pt-DPUCZDX8G_ISA0_B3136_MAX_BG2-1.3.1-r241.tar.gz
sudo tar -xvf ${HOME}/Downloads/pointpillars_kitti_12000_1_pt-DPUCZDX8G_ISA0_B3136_MAX_BG2-1.3.1-r241.tar.gz -C /usr/share/vitis_ai_library/models

echo "source ${HOME}/kv260_lidar_cam_fusion/install/setup.bash" >> ${HOME}/.bashrc

