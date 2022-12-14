# ROS trajectory tracking controller


## Goal of the project

This project provides a comparison between the ROS implementation of the Dynamic Window Approach algorithm
([`dwa_local_planner`](https://wiki.ros.org/dwa_local_planner) package) and a custom trajectory tracking controller.

The custom controller is composed of an inner linearisation law, based on the kinematic model, and an
outer tracking law, based on a proportional integral controller with velocity feed-forward.

More detailed information about the assignment can be found [here](material/project_assignment.pdf).


## Initial set up and compilation

Clone or download the repository into your home (`~`) folder.

If you use *bash*, add the following line to the end of your `.bashrc`:
```bash
source ~/ROS_trajectory_tracking_controller/devel/setup.bash
```

If you use *zsh*, add instead this other line to the end of your `.zshrc`:
```bash
source ~/ROS_trajectory_tracking_controller/devel/setup.zsh
```

Enter the project root directory and compile everything with:
```bash
cd ROS_trajectory_tracking_controller
catkin_make
```


## Run simulation and plot results

The following instructions let you perform and visualize a simulation.

### Trajectory control for differential drive

This simulation shows the behavior of the custom controller when an 8-shaped trajectory is set.

**[terminal #1]** Start the simulation:
```bash
roslaunch diffdrive_kin_ctrl diffdrive_kin_trajctrl.launch
```

**[terminal #2]** Enter the *script/* folder and record the simulation:
```bash
cd ~/ROS_trajectory_tracking_controller/src/diffdrive_kin_ctrl/script
rosbag record -a -O diffdrive_kin_trajctrl.bag
```

Wait about 30 seconds so that the simulation can be performed.

**[terminal #2]** Stop the recording with `Ctrl-C`.

**[terminal #1]** Stop the simulation with `Ctrl-C`.

**[terminal #2]** Visualize the results:
```bash
python plot_result_trajctrl.py diffdrive_kin_trajctrl.bag
```

### DWA for differential drive

This simulation shows the behavior of DWA when an 8-shaped trajectory is set.

**[terminal #1]** Start the simulation:
```bash
roslaunch diffdrive_dwa_ctrl diffdrive_dwa_trajctrl.launch
```

**[terminal #2]** Enter the *script/* folder and record the simulation:
```bash
cd ~/ROS_trajectory_tracking_controller/src/diffdrive_kin_ctrl/script
rosbag record -a -O diffdrive_dwa_trajctrl.bag
```

Wait about 30 seconds so that the simulation can be performed.

**[terminal #2]** Stop the recording with `Ctrl-C`.

**[terminal #1]** Stop the simulation with `Ctrl-C`.

**[terminal #2]** Visualize the results:
```bash
python plot_result_trajctrl.py diffdrive_dwa_trajctrl.bag
```

### Compare two simulations

If you wish, you can also compare the results of two different simulations.

Given two bag files named bag_1.bag and bag_2.bag, visualize the comparison:
```bash
python plot_result_comparison.py bag_1.bag bag_2.bag
```
