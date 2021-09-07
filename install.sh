
# upgrading python 3.5 to 3.6 (to be able to use harold to get cont tf fro discrete)
sudo add-apt-repository ppa:jonathonf/python-3.6

sudo apt-get update

sudo apt-get install python3.6

sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.6 1
sudo apt-get install python3.6-dev

# Needed for px4tools
sudo apt-get install libgeos-3.8.0
sudo apt-get install libgeos-dev
# Needed to be able to "show" plots by matplotlib
sudo apt-get install python3-tk

# Install SIPPY from github
git submodule update --init --recursive
cd SIPPY
sudo python3 setup.py install
