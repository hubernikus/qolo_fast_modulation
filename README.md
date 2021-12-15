# qolo_fast_modulation
QOLO Fast Modulation


# Clone and Setup
Clone the environment including submodules.
``` bash
git clone --recurse-submodules -j8 git://github.com/foo/bar.git

```

Install all requirements:
``` bash
pip install -r requirements.txt
```

Install the submodules:
``` r
cd scripts/vartools && python setup.py develop && ../..
cd scripts/fast_obstacle_avoidance && python setup.py develop && ../..
```



# Run Setup
A detailed description how to run the setup can be found here:
https://github.com/DrDiegoPaez/qolo_ros/blob/master/Guides/MDS_sequence_experiments.md


## Run this Repository
``` bash
source ~/autonomy_ws/src/qolo_fast_modulation/.venv/bin/activate
python ~/autonomy_ws/src/qolo_fast_modulation/scripts/controller_laserscan.py
```

