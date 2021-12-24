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
source ~/ROS_trajectory_tracking_controller/devel/setup.bash
```

Enter the project root directory and compile everything with:
```bash
cd ROS_trajectory_tracking_controller
catkin_make
```


## Run simulation and plot results

The following instructions let you perform and visualize a simulation.

### Linear control

This simulation shows the behavior of the controller in response to some simple velocity input commands.

**[terminal #1]** Start the simulation:
```bash
roslaunch unicycle_kin_ctrl unicycle_kin_linctrl.launch
```

**[terminal #2]** Enter the *script/* folder and record the simulation:
```bash
cd ~/ROS_trajectory_tracking_controller/src/unicycle_kin_ctrl/script
rosbag record -a -O unicycle_kin_linctrl.bag
```

Wait about 30 seconds so that the simulation can be performed.

**[terminal #2]** Stop the recording with `Ctrl-C`.

**[terminal #1]** Stop the simulation with `Ctrl-C`.

**[terminal #2]** Visualize the results:
```bash
python plot_result_linctrl.py unicycle_kin_linctrl.bag
```

### Trajectory control

This simulation shows the behavior of the controller when an 8-shaped trajectory is set.

**[terminal #1]** Start the simulation:
```bash
roslaunch unicycle_kin_ctrl unicycle_kin_trajctrl.launch
```

**[terminal #2]** Enter the *script/* folder and record the simulation:
```bash
cd ~/ROS_trajectory_tracking_controller/src/unicycle_kin_ctrl/script
rosbag record -a -O unicycle_kin_trjctrl.bag
```

Wait about 30 seconds so that the simulation can be performed.

**[terminal #2]** Stop the recording with `Ctrl-C`.

**[terminal #1]** Stop the simulation with `Ctrl-C`.

**[terminal #2]** Visualize the results:
```bash
python plot_result_trajctrl.py unicycle_kin_trjctrl.bag
```

