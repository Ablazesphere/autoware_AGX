# Autoware Installation Guide for NVIDIA AGX Orin

A comprehensive guide for installing Autoware on NVIDIA AGX Orin development kits.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Hardware Setup and OS Installation](#hardware-setup-and-os-installation)
- [Installation Steps](#installation-steps)
- [Post-Installation Setup](#post-installation-setup)
- [Verification](#verification)
- [Troubleshooting](#troubleshooting)
- [Performance Optimization](#performance-optimization)
- [Additional Resources](#additional-resources)

## Prerequisites

### Hardware Requirements
- **Device**: NVIDIA AGX Orin Developer Kit

### Software Requirements

#### Host Computer (for flashing)
- **Operating System**: Ubuntu 18.04 or later (20.04/22.04 recommended)
- **Available Storage**: Minimum 15GB free space for SDK Manager downloads
- **Internet Connection**: Stable broadband connection
- **USB Port**: USB-A or USB-C port for device connection

## Hardware Setup and OS Installation

### Step 1: AGX Orin Hardware Setup

#### 1.1 Connect the Hardware
1. **Power Connection**: Connect the power adapter to the AGX Orin Developer Kit
2. **USB Connection**: Connect a USB-C cable from your host computer to the USB-C port on the AGX Orin

#### 1.2 Enter Recovery Mode
1. Locate the **Force Recovery** button on the AGX Orin
2. Press and hold the **Force Recovery** button
3. While holding Force Recovery button, press and hold the **Power** button
4. Hold both the buttons for 2 more seconds, then release
5. The device is now in recovery mode

### Step 2: Install NVIDIA SDK Manager

#### 2.1 Download SDK Manager
1. Go to [NVIDIA Developer Portal](https://developer.nvidia.com/sdk-manager)
2. Download the latest SDK Manager for your host OS (Ubuntu recommended)
3. Install the downloaded package

#### 2.2 Launch SDK Manager
```bash
sdkmanager
```

### Step 3: Flash JetPack 6.2.1

#### 3.1 SDK Manager Configuration
1. **Login**: Sign in with your NVIDIA Developer account
2. **Product Category**: Select "Jetson"
3. **Hardware Configuration**: 
   - Target Hardware: AGX Orin Developer Kit
   - Target Operating System: Linux
4. **SDK Manager Options**:
   - JetPack Version: **6.2.1**
   - Target Components: **Jetson OS** (uncheck all SDK components)

#### 3.2 Component Selection
**IMPORTANT**: Only select the following components:
- ✅ **Jetson Linux**
- ✅ **Jetson OS**
- ❌ **Runtime Components** (uncheck)
- ❌ **SDK Components** (uncheck)
- ❌ **Platform Services** (uncheck)

#### 3.3 Installation Process
1. Click **Continue** to proceed
2. **Download & Install Options**:
   - Download folder: Choose appropriate location (requires ~10GB space)
   - Select "Download and install now"
3. **Flash Configuration**:
   - Runtime username: Create a username for the Jetson
   - Password: Set a secure password
   - Automatic login: Choose as preferred
4. Click **Flash** to begin the installation

#### 3.4 Monitor Installation
- The flashing process takes approximately 30-45 minutes
- Monitor progress in SDK Manager
- **Do not disconnect** the AGX Orin during flashing

#### 3.5 Complete Setup
1. Once flashing completes, the AGX Orin will automatically reboot
2. Follow the on-screen Ubuntu setup wizard on the connected display
3. Verify network connectivity

## Installation Steps

### Step 5: System Preparation
```bash
# [PLACEHOLDER - system update commands]
# Example:
sudo apt update && sudo apt upgrade -y
sudo apt install nvidia-jetpack
```

This will update all system packages to the latest versions and install NVIDIA JetPack components including CUDA 12.6 and TensorRT 10.3 that were skipped during the OS-only installation.

### Step 6: ROS2 Installation
Install ROS2 Humble using the development setup method, which provides the most flexibility for Autoware development. Follow the complete installation guide at https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html

### Step 9: Autoware Installation

Clone autowarefoundation/autoware and move to the directory.
```bash
git clone https://github.com/autowarefoundation/autoware.git
cd autoware
```
Open the ```universe.yaml``` file located at ```autoware/ansible/playbooks/universe.yaml``` and remove this highlighted line


![image](https://github.com/user-attachments/assets/89a59f19-b842-4341-8078-0b1266ef1590)
<br/>
<br/>
You can now directly execute ./setup-dev-env.sh, it will ask you whether to Install NVIDIA libraries, select no
```
./setup-dev-env.sh
```

Create the src directory and clone repositories into it.
```
cd autoware
mkdir src
vcs import src < autoware.repos
```

Install dependent ROS packages
```
source /opt/ros/humble/setup.bash
sudo apt update && sudo apt upgrade
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

Install and set up ccache to speed up consecutive builds
```
sudo apt update && sudo apt install ccache
```

Create the Ccache configuration folder and file:
```
mkdir -p ~/.cache/ccache
touch ~/.cache/ccache/ccache.conf
```

Set the maximum cache size. The default size is 5GB, but you can increase it depending on your needs. Here, we're setting it to 60GB
```
echo "max_size = 60G" >> ~/.cache/ccache/ccache.conf

```

To ensure Ccache is used for compilation, add the following lines to your .bashrc file. This will redirect GCC and G++ calls through Ccache
```
export CC="/usr/lib/ccache/gcc"
export CXX="/usr/lib/ccache/g++"
export CCACHE_DIR="$HOME/.cache/ccache/"
```

Build the workspace, Autoware uses colcon to build workspaces
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Step 10: Workspace Setup
```bash
# [PLACEHOLDER - workspace creation and build commands]
```

[PLACEHOLDER - workspace configuration details]

## Post-Installation Setup

### Environment Configuration
```bash
# [PLACEHOLDER - environment setup commands]
# Add to ~/.bashrc or ~/.zshrc
```

### AGX Orin Specific Optimizations
[PLACEHOLDER - any AGX Orin specific configurations]

## Verification

### Test Basic Installation
```bash
# [PLACEHOLDER - basic test commands]
```

### Run Sample Application
```bash
# [PLACEHOLDER - sample application commands]
```

**Expected Output:**
```
[PLACEHOLDER - expected output description]
```

## Troubleshooting

### Common Issues

#### Issue 1: AGX Orin Not Detected in Recovery Mode
**Problem**: SDK Manager cannot detect the AGX Orin device

**Solution**: 
```bash
# Verify USB connection and recovery mode
lsusb | grep NVIDIA

# If not detected, try:
# 1. Re-enter recovery mode (hold Force Recovery + press Reset)
# 2. Try different USB-C cable
# 3. Check USB port on host computer
# 4. Install missing USB drivers on host
```

#### Issue 2: SDK Manager Installation Fails
**Problem**: JetPack installation stops or fails during flashing

**Solution**: 
- Ensure stable internet connection
- Check available disk space (minimum 15GB free)
- Verify AGX Orin power connection is secure
- Do not use USB hubs; connect directly to host computer
- Try using a different USB-C cable

#### Issue 3: [PLACEHOLDER - common issue title]
**Problem**: [PLACEHOLDER - problem description]

**Solution**: 
```bash
# [PLACEHOLDER - solution commands]
```

#### Issue 2: [PLACEHOLDER - common issue title]
**Problem**: [PLACEHOLDER - problem description]

**Solution**: 
[PLACEHOLDER - solution steps]

### Performance Issues
- [PLACEHOLDER - performance troubleshooting tips]
- [PLACEHOLDER - memory optimization tips]
- [PLACEHOLDER - GPU utilization tips]

## Performance Optimization

### AGX Orin Power Modes
```bash
# [PLACEHOLDER - power mode configuration commands]
```

### Memory Management
[PLACEHOLDER - memory optimization recommendations]

### GPU Utilization
[PLACEHOLDER - GPU optimization settings]

## Additional Resources

- [Autoware Official Documentation](https://autowarefoundation.github.io/autoware-documentation/)
- [NVIDIA AGX Orin Developer Kit User Guide](https://developer.nvidia.com/embedded/jetson-agx-orin-developer-kit)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)

## Contributing

[PLACEHOLDER - contribution guidelines if this is a community project]

## License

[PLACEHOLDER - license information]

## Support

For issues and questions:
- [PLACEHOLDER - support contact information]
- [PLACEHOLDER - issue tracker links]

---

**Last Updated**: [PLACEHOLDER - date]
**Tested On**: NVIDIA AGX Orin Developer Kit with [PLACEHOLDER - specific software versions]
