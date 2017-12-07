# axxb_calibration

### Recommended setup
- Computer running **Ubuntu 16.04**, 64 bits.
- 1 x Denso VS060A3-AV6-NNN-NNNAN-NNNN

### Maintainers
[Huy Nguyen](https://dinhhuy2109.github.io/)

### Documentation
-Required packages:
+ [Denso common](https://github.com/quangounet/denso_common)
+ [ensenso](https://github.com/crigroup/ensenso)
+ [criros](https://github.com/crigroup/criros)

## Installation
### Robotics Workstation Setup in Ubuntu 16.04 (Xenial)

Install:
- OpenRAVE
- ROS Kinetic

http://fsuarez6.github.io/blog/workstation-setup-xenial/

### Ensenso SDK and uEye driver
To be able to use the Ensenso 3D camera you need the Ensenso SDK and uEye driver:
```bash
cd ~/Downloads
wget http://dl.ensenso.de/public/Software/EnsensoSDK/EnsensoSDK-1.3.180-x64.deb
wget http://dl.ensenso.de/public/IDS-Drivers/uEye_4.72_Linux_64.tgz
tar -xvzf uEye_4.72_Linux_64.tgz
cd uEye-Linux-4.72-64-bit; sudo ./ueyesdk-setup-4.72-eth-amd64.gz.run
cd ~/Downloads; sudo dpkg -i EnsensoSDK-1.3.180-x64.deb
```

### Packages from Ubuntu repositories
```bash
sudo apt-get install blender openscad python-rtree
```

### Python modules
You can install some modules using `pip`:
```bash
pip install cython control trimesh --user
```

And the rest need to be installed manually:
```bash
cd ~/git
git clone https://github.com/dinhhuy2109/python-cope.git
cd ~/git/python-cope && pip install -e . --user
```

### ROS Packages
```bash
cd ~/catkin_ws/src
wstool init .
wstool merge https://raw.github.com/crigroup/crigroup_docs/master/common.rosinstall
wstool update
git clone https://github.com/quangounet/denso_common.git
git clone https://github.com/dinhhuy2109/axxb_data_collection.git
```

Install any missing dependencies using `rosdep`:
```
rosdep update
rosdep install --from-paths . --ignore-src -y
``` 

Compile your ROS workspace:
```bash
cd ~/catkin_ws && catkin_make
catkin_make install
``` 
