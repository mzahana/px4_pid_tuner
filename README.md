# px4_pid_tuner
Python script for system identification and tuning of PX4 PID loops based on PX4 logs (ulog only).

It currently tunes the attitude rate loops only, `ROLL_RATE_P`/I/D gains. Similarly, for pitch/yaw. Future updates will allow attitude loop P gain tuning as well as translational velocity and position loops.

# Background
The python script performs two main tasks.
1. Identifies a **2nd order system** which will be used for PID tuning. This is done using [SIPPY](https://github.com/CPCLAB-UNIPI/SIPPY) package.
2. Given the model identified in 1, it performs LQR-based PID tuning as described in [this paper](https://ieeexplore.ieee.org/document/4242954). In LQR-based tuning, the PID gains are optimal given specific LQR weight matrices, Q and R. To find best Q and R matrices, genetic optimization is employed using [DEAP](https://deap.readthedocs.io/en/master/overview.html) python package

# Installation
Look at the required modules in the `install.sh` file.

# Usage
The script is called from the command line with positional arguments as follows.
* To only show input/output data for inspection before identification, you can use the `-sd true` or `--showDataOnly true` argument.
```
python3 px4_pid_tuner.py -f log.ulg -sd true
```

* To select which axis i.e. roll/pitch/yaw, use the `-a` or `--axis`
```
python3 px4_pid_tuner.py -f log.ulg -a roll -sd true
```
If not provided, the default is roll

* To select data range (start/end time in seconds), use the `-s` for start and `-e` for end times. The time is in seconds
```
python3 px4_pid_tuner.py -f log.ulg -s 2.0 -e 5.5 -sd true
```

* If `-sd true` is not set, system identificaion and PID tuning will be performed. In addition, the data will be divided into mini-batches to find a data range with best-fit model. To disable the automatic data range selection, set  autoSelect `-as false` and provide range of data manually using `-s <start_time>` and `-e <end_time>`. For example,
```
python3 px4_pid_tuner.py -f log.ulg -as false -s 2.0 -e 5.5
```

* If you want the script to try to automatically select data range with best fit, set `-as true` and determine the batch size using `-bs 4.0` (4 seconds in this case).
```
python3 px4_pid_tuner.py -f log.ulg -as true -bs 4.0
```

* To determine the resampling time of i/o data, use `-dt 0.001`. Sampling time is in seconds.
```
python3 px4_pid_tuner.py -f log.ulg -as false -s 2.0 -e 5.5 -ddt 0.001
```

# Notes

* The quality of system identification is highly dependent on the input/output data coming from the `ulg` file. If the system has poor data i.e. no excitation in different axes with different frequencies, the model can be off and PID tuning results will be useless.
* Make sure that you use the good portions of your data (ones with enough excitation), and one where the controller is active (outputing signal).
* The automatics data range selector (`-as true`) is not yet optimized, so, use common sense to judge if the resulting batch of the automatic range selector is really good portion of the data.

# TODO
- [ ] Do more data pre-procssing to get better identified system.
- [ ] Implement a function similar to MATLAB function `advice()` (https://www.mathworks.com/help/ident/ref/advice.html). This will help to automatically analyze I/O data and provide recommendations for better model identification.
- [ ] Allow user to graphically select range of data from a plot for system identification
- [ ] Implement online optimal PID tuning !