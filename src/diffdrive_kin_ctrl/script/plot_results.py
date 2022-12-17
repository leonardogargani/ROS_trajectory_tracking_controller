import sys
import rosbag
import matplotlib.pyplot as plt

# Read data from bag
bag = rosbag.Bag(sys.argv[1])

# State published by the simulator
vehicleState_time = []
vehicleState_x = []
vehicleState_y = []
vehicleState_theta = []
vehicleState_angularvelocity_r = []
vehicleState_angularvelocity_l = []

# State published by the controller
controllerState_time = []
controllerState_xref = []
controllerState_yref = []
controllerState_xPref = []
controllerState_yPref = []
controllerState_xP = []
controllerState_yP = []
controllerState_vPx = []
controllerState_vPy = []
controllerState_angularvelocity = []
controllerState_linearvelocity = []
controllerState_angularvelocity_r = []
controllerState_angularvelocity_l = []
controllerState_xPerr = []
controllerState_yPerr = []

# Look into only relevant topics
for topic, msg, t in bag.read_messages():

	if topic == "/robot_state":
		vehicleState_time.append(msg.data[0])
		vehicleState_x.append(msg.data[1])
		vehicleState_y.append(msg.data[2])
		vehicleState_theta.append(msg.data[3])
		vehicleState_angularvelocity_r.append(msg.data[4])
		vehicleState_angularvelocity_l.append(msg.data[5])

	if topic == "/controller_state":
		controllerState_time.append(msg.data[0])
		controllerState_xref.append(msg.data[1])
		controllerState_yref.append(msg.data[2])
		controllerState_xPref.append(msg.data[3])
		controllerState_yPref.append(msg.data[4])
		controllerState_xP.append(msg.data[5])
		controllerState_yP.append(msg.data[6])
		controllerState_vPx.append(msg.data[7])
		controllerState_vPy.append(msg.data[8])
		controllerState_linearvelocity.append(msg.data[9])
		controllerState_angularvelocity.append(msg.data[10])
		controllerState_angularvelocity_r.append(msg.data[11])
		controllerState_angularvelocity_l.append(msg.data[12])
		controllerState_xPerr.append(msg.data[3]-msg.data[5])
		controllerState_yPerr.append(msg.data[4]-msg.data[6])

bag.close()

# Plot eight-shaped reference trajectory and real trajectory
fig_traj, ax_traj = plt.subplots()
fig_traj.canvas.set_window_title('Trajectory')
ax_traj.plot(vehicleState_x,vehicleState_y, label="actual")
ax_traj.plot(vehicleState_x[0],vehicleState_y[0],'ro')
ax_traj.plot(vehicleState_x[len(vehicleState_x)-1],vehicleState_y[len(vehicleState_x)-1],'rx')
ax_traj.plot(controllerState_xref,controllerState_yref,'g', label="reference")
ax_traj.set(xlabel='x [m]', ylabel='y [m]')
ax_traj.legend(loc='best')

# Plot velocities (linear and angular) produced by dwa_local_planner
fig_dwa_vel, (ax_dwa_vel_1, ax_dwa_vel_2) = plt.subplots(2)
fig_dwa_vel.canvas.set_window_title('DWA Velocities')
ax_dwa_vel_1.plot(controllerState_time,controllerState_linearvelocity)
ax_dwa_vel_1.set_ylim(-0.01,0.2)
ax_dwa_vel_1.set(xlabel='Time [s]', ylabel='Linear velocity [m/s]')
ax_dwa_vel_2.plot(controllerState_time,controllerState_angularvelocity)
ax_dwa_vel_2.set(xlabel='Time [s]', ylabel='Angular velocity [rad/s]')
ax_dwa_vel_2.set_ylim(-0.8,0.8)

# Plot right and left wheels' velocities
fig_wvel, (ax_wvel_1, ax_wvel_2) = plt.subplots(2)
fig_wvel.canvas.set_window_title('Wheels Velocities')
ax_wvel_1.plot(vehicleState_time,vehicleState_angularvelocity_r)
ax_wvel_1.set(xlabel='Time [s]', ylabel='Right angular velocity [rad/s]')
ax_wvel_2.plot(vehicleState_time,vehicleState_angularvelocity_l)
ax_wvel_2.set(xlabel='Time [s]', ylabel='Left angular velocity [rad/s]')

# Plot actual position of the robot and reference position
fig_pose, (ax_pose_1, ax_pose_2, ax_pose_3) = plt.subplots(3)
fig_pose.canvas.set_window_title('Pose Comparison')
ax_pose_1.plot(vehicleState_time,vehicleState_x, label="x act")
ax_pose_1.plot(controllerState_time,controllerState_xref, 'r--', label="x ref")
ax_pose_1.set(xlabel='Time [s]', ylabel='x [m]')
ax_pose_1.legend(loc='best')
ax_pose_2.plot(vehicleState_time,vehicleState_y, label="y act")
ax_pose_2.plot(controllerState_time,controllerState_yref, 'r--', label="y ref")
ax_pose_2.set(xlabel='Time [s]', ylabel='y [m]')
ax_pose_2.legend(loc='best')
ax_pose_3.plot(vehicleState_time,vehicleState_theta)
ax_pose_3.set(xlabel='Time [s]', ylabel='theta [rad]')

# Plot position error
fig_err, (ax_err_1, ax_err_2) = plt.subplots(2)
fig_err.canvas.set_window_title('Position Error')
ax_err_1.plot(controllerState_time,controllerState_xPerr)
ax_err_1.set(xlabel='Time [s]', ylabel='x position error [m]')
ax_err_2.plot(controllerState_time,controllerState_yPerr)
ax_err_2.set(xlabel='Time [s]', ylabel='y position error [m]')

plt.show()

