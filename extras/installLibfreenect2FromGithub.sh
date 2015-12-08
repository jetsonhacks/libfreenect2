#!/bin/sh

# Remove leftovers from previous builds
sudo rm -rf libfreenect2

# Get libfreenect2 and checkout current branch(they could be incompatible)
git clone https://github.com/jetsonhacks/libfreenect2.git -b jetsontk1-L4T-21.3

./installLibfreenect2FromDir.sh libfreenect2
