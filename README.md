# Autoware Installation Guide for NVIDIA AGX Orin

A comprehensive guide for installing Autoware on NVIDIA AGX Orin development kits.

## Version Compatibility

| Autoware Version | Status | Notes |
|------------------|--------|-------|
| 0.44.2 | ✅ Tested & Working | This guide is verified with this version |
| 0.45.x | ⚠️ Not Tested | Compatibility unknown |

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

### Step 3: Flash JetPack 6.2

#### 3.1 SDK Manager Configuration
1. **Login**: Sign in with your NVIDIA Developer account
2. **Product Category**: Select "Jetson"
3. **Hardware Configuration**: 
   - Target Hardware: AGX Orin Developer Kit
   - Target Operating System: Linux
4. **SDK Manager Options**:
   - JetPack Version: **6.2**
   - Target Components: **Jetson OS** (uncheck all SDK components)

#### 3.2 Component Selection
**IMPORTANT**: Select ALL the following components for complete Autoware compatibility:
- ✅ **Jetson Linux**
- ✅ **Jetson Runtime Components**
- ✅ **Jetson SDK Components**
- ✅ **Jetson Platform Services**

#### 3.3 Installation Process
1. Click **Continue** and follow the SDK Manager prompts to flash JetPack 6.2 to your AGX Orin (this process takes 30-45 minutes).

## Installation Steps

### Step 4: OpenCV Version Downgrade (Critical Step)

**IMPORTANT**: The newer OpenCV version introduces incompatible function signatures and deprecated features that break Autoware's build system. To resolve this, we remove JetPack's OpenCV installation and downgrade to the Ubuntu repository's 4.5.4 version that Autoware was designed to work with.

```bash
# Remove all OpenCV packages
sudo apt remove --purge libopencv* opencv* python3-opencv

# Clean up any remaining dependencies
sudo apt autoremove
sudo apt autoclean

# Update package lists
sudo apt update

# Install OpenCV 4.5.4 (required for Autoware compatibility)
sudo apt install libopencv-dev=4.5.4+dfsg-9ubuntu4 python3-opencv=4.5.4+dfsg-9ubuntu4
```

**Note**: This downgrade is essential. Skipping this step will result in Autoware build failures.

### Step 5: System Preparation
```bash
# Update system packages
sudo apt update && sudo apt upgrade -y
```

Ensure your system is fully updated before proceeding with ROS2 and Autoware installation.

### Step 6: ROS2 Humble Installation

Install ROS2 Humble by following the official installation guide: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

### Step 7: Autoware Installation

#### 8.1 Clone Autoware Repository

Clone Autoware version 0.44.2 and navigate to the directory:

```bash
git clone https://github.com/autowarefoundation/autoware.git -b 0.44.2
cd autoware
```

#### 8.2 Modify Universe Configuration

**CRITICAL**: Before running the setup script, you must modify the universe.yaml file to prevent conflicts:

1. Open the universe.yaml file:
```bash
nano autoware/ansible/playbooks/universe.yaml
```

2. Remove the highlighted line as shown in the image reference
   - Look for and remove any conflicting library specifications

#### 8.3 Run Development Environment Setup

Execute the setup script and **select "no"** when asked about installing NVIDIA libraries (since we already have them from JetPack):

```bash
./setup-dev-env.sh
```

**Important**: When prompted "Install NVIDIA libraries?", answer **no**.

#### 8.4 Import Source Repositories

Create the src directory and import all Autoware repositories:

```bash
mkdir src
vcs import src < autoware.repos
```

#### 8.5 Install ROS Dependencies

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Update packages and install dependencies
sudo apt update && sudo apt upgrade
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

#### 8.6 Setup Build Acceleration with ccache

Install and configure ccache to speed up consecutive builds:

```bash
# Install ccache
sudo apt update && sudo apt install ccache

# Create ccache configuration
mkdir -p ~/.cache/ccache
touch ~/.cache/ccache/ccache.conf

# Set cache size to 60GB (adjust based on available storage)
echo "max_size = 60G" >> ~/.cache/ccache/ccache.conf
```

Add ccache environment variables to your shell configuration:

```bash
# Add to ~/.bashrc
echo 'export CC="/usr/lib/ccache/gcc"' >> ~/.bashrc
echo 'export CXX="/usr/lib/ccache/g++"' >> ~/.bashrc
echo 'export CCACHE_DIR="$HOME/.cache/ccache/"' >> ~/.bashrc

# Source the updated bashrc
source ~/.bashrc
```

#### 8.7 Build Autoware

Build the complete Autoware workspace:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**Note**: This build process may take 1-3 hours depending on your AGX Orin configuration and available resources.
