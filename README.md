# GTSAM-tutorial
GTSAM hands on session based on the official tutorial

# Installation

Install GTSAM
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