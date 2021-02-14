#! /bin/bash
# DONT RUN AS ROOT
# tested on ubuntu 18.04

if [ `whoami` == 'root' ]; then
	echo "don't run this as root"
	exit 1
fi

# ensure the script exits on error
set -e

sudo apt update
sudo apt install libgeos-3* libgeos-dev python3-tk gfortran cmake libssl-dev libblas-dev liblapack-dev -y
sudo apt install python3-pip python3-numpy -y
sudo apt install python3-setuptools -y

#pip3 install numpy --user

pip3 install https://github.com/matplotlib/basemap/archive/master.zip  --user

pip3 install px4tools --user
pip3 install harold --user

# Dependencies for SIPPY
pip3 install control --user
pip3 install scipy --user

# needed for slycot
pip3 install scikit-build --user
pip3 install slycot --user
pip3 install future --user
pip3 install matplotlib --user

# Genetic algorithm package
pip3 install deap --user


# Install SIPPY from github
git submodule init 
git submodule update 
cd SIPPY
python3 setup.py install --user
cd -


