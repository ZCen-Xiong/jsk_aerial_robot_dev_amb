#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO:-one}"
ROS_APT_DISTRO="${ROS_DISTRO_NAME}"

if [[ "${ROS_DISTRO_NAME}" == "debian" ]] && apt-cache show ros-one-realsense2-camera >/dev/null 2>&1; then
  ROS_APT_DISTRO="one"
fi

ROS_RS_PACKAGE="ros-${ROS_APT_DISTRO}-realsense2-camera"
ROS_LRS_PACKAGE="ros-${ROS_APT_DISTRO}-librealsense2"

echo "Detected ROS_DISTRO=${ROS_DISTRO_NAME}; using apt ROS distro '${ROS_APT_DISTRO}'."
echo "Updating apt package lists..."
sudo apt-get update

echo "Installing RealSense ROS wrapper packages..."
sudo apt-get install -y --no-install-recommends \
  "${ROS_LRS_PACKAGE}" \
  "${ROS_RS_PACKAGE}"

if apt-cache show librealsense2-utils >/dev/null 2>&1; then
  echo "Installing Intel librealsense runtime, udev rules, DKMS, viewer, and dev headers..."
  sudo apt-get install -y --no-install-recommends \
    librealsense2 \
    librealsense2-udev-rules \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev
else
  echo "Intel librealsense apt packages are not available in the current apt sources."
  echo "The ROS packages above include librealsense runtime support for realsense2_camera."
fi

echo
echo "Installation finished."
echo "Reconnect the D435i, then verify with:"
echo "  roslaunch amoeba d435i_realsense.launch"
echo
echo "If installed, SDK tools can be checked with:"
echo "  rs-enumerate-devices"
echo "  realsense-viewer"
