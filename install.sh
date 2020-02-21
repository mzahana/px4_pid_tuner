#! /bin/bash
# DONT RUN AS ROOT

set -e

sudo apt install libgeos-3* libgeos-dev python3-tk gfortran cmake libssl-dev libblas-dev liblapack-dev -y

pip3 install https://github.com/matplotlib/basemap/archive/master.zip

pip3 install numpy
pip3 install px4tools

pip3 install harold

# Dependencies for SIPPY
pip3 install control
pip3 install scipy

# needed for slycot
pip3 install scikit-build
pip3 install slycot
pip3 install future
pip3 install matplotlib

# Genetic algorithm package
pip3 install deap


# Install SIPPY from github
git submodule init
git submodule update
cd SIPPY
python3 setup.py install
cd -


