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
sudo apt-get install -y libgeos-3* libgeos-c1* libgeos-dev
sudo apt-get install python3-tk gfortran cmake libssl-dev libblas-dev liblapack-dev -y
sudo apt-get install python3-pip python3-numpy -y
sudo apt-get install python3-setuptools -y
#sudo apt-get install -y python3-mpltoolkits.basemap

pip3 install Cython --user
# pyproj
pip3 install --upgrade --user matplotlib numpy
#pip3 install https://github.com/matplotlib/basemap/archive/master.zip  --user

# needed for slycot
pip3 install scikit-build --user
#pip3 install slycot --user
pip3 install --user slycot==0.3.5.0
pip3 install future --user

pip3 install px4tools --user
pip3 install harold --user

# Dependencies for SIPPY
pip3 install control --user
pip3 install scipy --user


# Genetic algorithm package
pip3 install deap --user

pip3 uninstall pandas
# There is an issue with later versions of pandas
pip3 install pandas==0.25.3 --user


# Install SIPPY from github
git submodule init 
git submodule update 
cd SIPPY
python3 setup.py install --user
cd -

