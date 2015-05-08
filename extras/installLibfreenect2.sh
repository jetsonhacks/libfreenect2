#!/bin/sh
#from https://github.com/xlz/libfreenect2/wiki/Jetson-TK1-HOWTO

# Install Prerequisites
#opencv-dev -> opencv4tegra-dev -> already installed from Jetpack
sudo apt-get install -y build-essential libturbojpeg libtool autoconf libudev-dev cmake mesa-common-dev \
    freeglut3-dev libxrandr-dev doxygen libxi-dev libjpeg-turbo8-dev checkinstall

# Remove leftovers from previous builds
sudo rm -rf libfreenect2

# Get libfreenect2
git clone https://github.com/jetsonhacks/libfreenect2.git

# Copy nv_headers and turbojepg.h into libfreenect2/depends
wget http://developer.download.nvidia.com/mobile/tegra/l4t/r21.2.0/sources/gstjpeg_src.tbz2
tar -xvf gstjpeg_src.tbz2 gstjpeg_src/nv_headers
mv gstjpeg_src/nv_headers/ libfreenect2/depends/

# Cleanup the empty folder
rmdir gstjpeg_src/

# build libusb and glfw
cd libfreenect2/depends
sh install_ubuntu.sh

#fix up libturbojpeg.so linker doesn't get upset
sudo ln -s /usr/lib/arm-linux-gnueabihf/libturbojpeg.so.0.0.0 /usr/lib/arm-linux-gnueabihf/libturbojpeg.so

# Build Protonect example
cd ../examples/protonect/
mkdir build
cd build
cmake ..
make && sudo checkinstall -y --pkgname=libfreenect2 --exclude /home make install

# Install the udev rule so that you always have read/write permission to the Kinect V2
cd ../..
sudo cp extras/90-kinect2.rules /etc/udev/rules.d/90-kinect2.rules
/bin/echo -e "\e[1;32mFinished.\e[0m"
