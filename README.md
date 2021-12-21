# qolo_fast_modulation
QOLO Fast Modulation


# Clone and Setup
Clone the environment including submodules.
``` bash
git clone --recurse-submodules -j8 git://github.com/foo/bar.git

```

Setup virtual environment (use whatever compatible environment manager that you have with Python >3.7).

``` bash
python3.7 -m venv .venv && source .venv/bin/activate
```

Install all requirements:
``` bash
pip install -r requirements.txt
```

Install the submodules:
``` bash
cd scripts/vartools && python setup.py develop && ../..
cd scripts/fast_obstacle_avoidance && python setup.py develop && ../..
```
<!-- cd scripts/fast_obstacle_avoidance && python setup.py develop && ../.. -->



# Run Setup
A detailed description how to run the setup can be found here:
https://github.com/DrDiegoPaez/qolo_ros/blob/master/Guides/MDS_sequence_experiments.md


## Run this Repository (after setting up the evnironment
``` bash
source ~/autonomy_ws/src/qolo_fast_modulation/.venv/bin/activate
python ~/autonomy_ws/src/qolo_fast_modulation/scripts/controller_laserscan.py
```


# Error Bug / Issues
-> check that all breaks are released (!)
Release them by pressing at both extreme ends of the torso-force sensors simultaneously (!)

-> One wheel is not spinning
Make the wheels spin freely (detach the wheels); send full velocity command to both wheels for a few seconds (e.g. use a script for this)
``` bash
TODO
```



