# GTSAM-tutorial
GTSAM hands on session based on the official tutorial

# Installation

Requirements:
Boost (`sudo apt-get install libboost-all-dev`)
CMake (`sudo apt-get install cmake`)

GTSAM from Ubuntu PPA:
```
# Add PPA
sudo add-apt-repository ppa:borglab/gtsam-develop
sudo apt update  # not necessary since Bionic
# Install:
sudo apt install libgtsam-dev libgtsam-unstable-dev
```


Or install from source (not recommended atm, takes ~20-30mins)
GTSAM from Source
```
cd ~/
git clone https://github.com/borglab/gtsam.git
cd gtsam
mkdir build && cd build
```
If you have ROS:
```
cmake -DGTSAM_USE_SYSTEM_EIGEN=ON .. 
sudo make install
```
If you don't have ROS
```
cmake ..
sudo make install
```
# Building GTSAM-tutorial
```
mkdir -p ~/projects/hands-on/
cd ~/projects/hands-on/
git clone https://github.com/husseinalijaafar/GTSAM-tutorial.git
mkdir build && cd build
cmake ..
sudo make install
```

## Problems
If you encounter Eigen problems (although you probably wont)
Try installing Eigen separately 
`sudo apt install libeigen3-dev`


# References
This tutorial is basically a modified implementation of the GTSAM documentation tutorial, with added information and explanation.
Please see the tutorial for the original content [Factor Graphs and GTSAM](https://gtsam.org/tutorials/intro.html)