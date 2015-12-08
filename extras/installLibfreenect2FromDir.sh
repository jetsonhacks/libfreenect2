#!/bin/sh
#from https://github.com/xlz/libfreenect2/wiki/Jetson-TK1-HOWTO

if [ -z "$1" ]; then
    echo "Libfreenect2 project directory must be provided"
    exit 1
fi
LIBFREENECT2_DIR=$1

# Install Prerequisites
#opencv-dev -> opencv4tegra-dev -> already installed from Jetpack 
sudo apt-get install -y build-essential libturbojpeg libtool autoconf libudev-dev cmake mesa-common-dev \
    freeglut3-dev libxrandr-dev doxygen libxi-dev libjpeg-turbo8-dev checkinstall

# Copy nv_headers into libfreenect2/depends/gstjpeg_src
rm gstjpeg_src.tbz2
wget http://developer.download.nvidia.com/embedded/L4T/r21_Release_v3.0/sources/gstjpeg_src.tbz2
tar -xvf gstjpeg_src.tbz2 gstjpeg_src/nv_headers

rm -rf $LIBFREENECT2_DIR/depends/gstjpeg_src
mv gstjpeg_src $LIBFREENECT2_DIR/depends/

# build libusb and glfw
cd $LIBFREENECT2_DIR/depends
sh install_ubuntu.sh

#fix up libturbojpeg.so linker doesn't get upset
sudo ln -s /usr/lib/arm-linux-gnueabihf/libturbojpeg.so.0.0.0 /usr/lib/arm-linux-gnueabihf/libturbojpeg.so

# Build Protonect example
cd ../examples/protonect/
rm -rf build
mkdir build
cd build
cmake ..
make && sudo checkinstall -y --pkgname=libfreenect2 --exclude /home make install

# Install the udev rule so that you always have read/write permission to the Kinect V2
cd ../../..
sudo cp extras/90-kinect2.rules /etc/udev/rules.d/90-kinect2.rules
/bin/echo -e "\e[1;32mFinished.\e[0m"
