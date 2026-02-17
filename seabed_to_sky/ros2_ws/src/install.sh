#!/bin/bash
set -e
colcon build
source install/setup.bash

pip3 install numpy==1.21.0
pip3 install cv-bridge
pip3 install opencv-python
pip3 install scipy
pip3 install matplotlib
pip3 install scikit-image
pip3 install scikit-learn
pip3 install kappe
pip3 install nbformat
pip3 install plotly
pip3 install jupyter
pip3 install jupyterlab

# Add sourcing to .bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

export PATH=$PATH:/home/rosdev/.local/bin