
# upgrading python 3.5 to 3.6 (to be able to use harold to get cont tf fro discrete)
sudo add-apt-repository ppa:jonathonf/python-3.6

sudo apt-get update

sudo apt-get install python3.6

sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6 1
sudo apt-get install python3.6-dev

# Needed for px4tools
sudo apt-get install libgeos-3.5.0
sudo apt-get install libgeos-dev
sudo pip install https://github.com/matplotlib/basemap/archive/master.zip

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
# Needed to be able to "show" plots by matplotlib
sudo apt-get install python3-tk

# Install SIPPY from github
git clone https://github.com/CPCLAB-UNIPI/SIPPY.git
cd SIPPY
python3 setup.py install
# or, in case of permission error
sudo python setup.py install

# Genetic algorithm package
pip install deap
# or sudo pip install deap
