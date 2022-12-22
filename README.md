# ROS trajectory tracking controller


## Goal of the project

This project provides a comparison between the ROS implementation of the Dynamic Window Approach algorithm
([`dwa_local_planner`](https://wiki.ros.org/dwa_local_planner) package) and a custom trajectory tracking controller.

The custom controller is composed of an inner linearisation law, based on the kinematic model, and an
outer tracking law, based on a proportional integral controller with velocity feed-forward.

More detailed information about the assignment can be found [here](material/project_assignment.pdf).

