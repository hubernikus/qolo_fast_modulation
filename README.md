# qolo_fast_modulation
QOLO Fast Modulation


# Clone and Setup
Clone the environment including submodules (wihtout ssh).
``` bash
git clone --recurse-submodules -j8 https://github.com/hubernikus/qolo_fast_modulation
```

Setup virtual environment (use whatever compatible environment manager that you have with Python >3.7).

``` bash
python -m venv .venv && source .venv/bin/activate
```
with python -V > 3.7

Install all requirements:
``` bash
pip install -r requirements.txt
```
make sure you also install the submodules (mainly `vartools`)

Install the submodules:
``` bash
cd scripts/vartools && python setup.py develop && ../..
cd scripts/fast_obstacle_avoidance && python setup.py develop && ../..
```
<!-- cd scripts/fast_obstacle_avoidance && python setup.py develop && ../.. -->

## Run simulation (RVIZ based)
Launch visualization with qolo integrator
``` bash
roslaunch qolo_fastmod qolo_visualization.launch integrate_qolo_tf:="true"
```

Alternatively run  visualization and qolo-integrator separately:
``` bash
roslaunch qolo_fastmod qolo_visualization.launch
```

``` bash
source .venv/bin/activate
python scripts/node_qolo_trafo.py
```

Run the controller
``` bash
source .venv/bin/activate
python scripts/controller_basic_ds.py 
```


# Run Setup
A detailed description how to run the setup can be found here:
https://github.com/DrDiegoPaez/qolo_ros/blob/master/Guides/MDS_sequence_experiments.md


## Run this Repository (after setting up the environment
The startup on the QOLO computer `192.168.13.120`
``` bash
source ~/autonomy_ws/src/qolo_fast_modulation/.venv/bin/activate
python ~/autonomy_ws/src/qolo_fast_modulation/scripts/controller_laserscan.py
```

## To Run the QOLO with the Belt Controller
Change on the computer `110` in the `qolo` > `script` > `compliant_mds_shared_qolo.sh` the flags  
`remote_mode` := `false`  
`mds_shared_mode` :=  `true`  

The remote control can be access from http://192.168.13.110:8080/

# Error Bug / Issues
-> check that all breaks are released (!), check the topic `qolo/emergency` (True -> break is activated)
Release them by pressing at both extreme ends of the torso-force sensors simultaneously (the battery lights should flash when done succesfully)


-> One wheel is not spinning
 Make the wheels spin freely (detach the wheels); send full velocity command to both wheels for a few seconds (e.g. use a script for this)  
[WARNING] make sure the gears are deactivated, otherwise a crash might occur(!)  
``` bash
python wheels_recalibration.py
```

