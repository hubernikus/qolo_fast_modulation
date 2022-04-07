#!/bin/bash
# Setup all submodules
pip install -r src/dynamic_obstacle_avoidance/requirements.txt
pip install -e ./src/dynamic_obstacle_avoidance

pip install -r src/fast_obstacle_avoidance/requirements.txt
pip install -e ./src/fast_obstacle_avoidance/

pip install -r src/various_tools/requirements.txt
pip install -e ./src/various_tools/
