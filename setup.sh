#!/bin/bash

# Setup all submodules
pip install -r requirements.txt

# Dynamic Obstacle Avdoiance
pip install -r src/dynamic_obstacle_avoidance/requirements.txt
pip install -e ./src/dynamic_obstacle_avoidance

# Fast Obstacle Avoidance (Sensor Based)
pip install -r src/fast_obstacle_avoidance/requirements.txt
pip install -e ./src/fast_obstacle_avoidance/

# Various Mathematical Tools
pip install -r src/various_tools/requirements.txt
pip install -e ./src/various_tools/
