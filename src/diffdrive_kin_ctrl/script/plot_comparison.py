import sys
import rosbag
import matplotlib.pyplot as plt

# Read data from bag
bag_1 = rosbag.Bag(sys.argv[1])
bag_2 = rosbag.Bag(sys.argv[2])

fig_traj, ax_traj = plt.subplots()
fig_dwa_vel, (ax_dwa_vel_1, ax_dwa_vel_2) = plt.subplots(2)
fig_wvel, (ax_wvel_1, ax_wvel_2) = plt.subplots(2)
fig_pose, (ax_pose_1, ax_pose_2, ax_pose_3) = plt.subplots(3)
fig_err, (ax_err_1, ax_err_2) = plt.subplots(2)

controllerState_xref = []
controllerState_yref = []

for bag in [bag_1, bag_2]:

	bag_name = bag.filename

	# State published by the simulator ("actual")
	vehicleState_time = []
	vehicleState_x = []
	vehicleState_y = []
	vehicleState_theta = []
	vehicleState_angularvelocity_r = []
	vehicleState_angularvelocity_l = []

	# State published by the controller ("reference")
	controllerState_time = []
	controllerState_xref_tmp = []
	controllerState_yref_tmp = []
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
	
	
	for topic, msg, _ in bag.read_messages():
		if topic == '/robot_state':
			vehicleState_time.append(msg.data[0])
			vehicleState_x.append(msg.data[1])
			vehicleState_y.append(msg.data[2])
			vehicleState_theta.append(msg.data[3])
			vehicleState_angularvelocity_r.append(msg.data[4])
			vehicleState_angularvelocity_l.append(msg.data[5])

		if topic == '/controller_state':
			controllerState_time.append(msg.data[0])
			controllerState_xref_tmp.append(msg.data[1])
			controllerState_yref_tmp.append(msg.data[2])
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
	
	# Plot eight-shaped reference trajectory and actual trajectory
	fig_traj.canvas.set_window_title('Trajectory')
	ax_traj.plot(vehicleState_x,vehicleState_y, label="actual " + bag_name)
	ax_traj.plot(vehicleState_x[0],vehicleState_y[0],'ro')
	ax_traj.plot(vehicleState_x[len(vehicleState_x)-1],vehicleState_y[len(vehicleState_x)-1],'rx')
	
	if len(controllerState_xref_tmp) > len(controllerState_xref):
		controllerState_xref = controllerState_xref_tmp
		controllerState_yref = controllerState_yref_tmp
		
	ax_traj.set(xlabel='x [m]', ylabel='y [m]')
	
	# Plot velocities (linear and angular) produced by dwa_local_planner
	fig_dwa_vel.canvas.set_window_title('DWA Velocities')
	ax_dwa_vel_1.plot(controllerState_time,controllerState_linearvelocity, label=bag_name)
	ax_dwa_vel_1.set_ylim(-0.01,0.2)
	ax_dwa_vel_1.set(xlabel='Time [s]', ylabel="Linear velocity [m/s]")
	ax_dwa_vel_1.legend(loc='best')
	ax_dwa_vel_2.plot(controllerState_time,controllerState_angularvelocity, label=bag_name)
	ax_dwa_vel_2.set(xlabel='Time [s]', ylabel="Angular velocity [rad/s]")
	ax_dwa_vel_2.set_ylim(-0.8,0.8)
	ax_dwa_vel_2.legend(loc='best')
	
	
	# Plot right and left wheels' velocities
	fig_wvel.canvas.set_window_title('Wheels Velocities')
	ax_wvel_1.plot(vehicleState_time,vehicleState_angularvelocity_r, label=bag_name)
	ax_wvel_1.set(xlabel='Time [s]', ylabel='Right angular velocity [rad/s]')
	ax_wvel_1.set_ylim(-1.0,8.0)
	ax_wvel_1.legend(loc='best')
	ax_wvel_2.plot(vehicleState_time,vehicleState_angularvelocity_l, label=bag_name)
	ax_wvel_2.set(xlabel='Time [s]', ylabel='Left angular velocity [rad/s]')
	ax_wvel_2.set_ylim(-1.0,8.0)
	ax_wvel_2.legend(loc='best')
	
	
	# Plot actual position of the robot and reference position
	fig_pose.canvas.set_window_title('Pose Comparison')
	ax_pose_1.plot(vehicleState_time,vehicleState_x, label=bag_name)
	#ax_pose_1.plot(controllerState_time,controllerState_xref, 'k--', label="x ref")
	ax_pose_1.set(xlabel='Time [s]', ylabel='x [m]')
	#ax_pose_1.legend(loc='best')
	ax_pose_2.plot(vehicleState_time,vehicleState_y, label=bag_name)
	#ax_pose_2.plot(controllerState_time,controllerState_yref, 'k--', label="y ref")
	ax_pose_2.set(xlabel='Time [s]', ylabel='y [m]')
	#ax_pose_2.legend(loc='best')
	ax_pose_3.plot(vehicleState_time,vehicleState_theta, label=bag_name)
	ax_pose_3.set(xlabel='Time [s]', ylabel='theta [rad]')
	ax_pose_3.legend(loc='best')
	
	# Plot position error
	fig_err.canvas.set_window_title('Position Error')
	ax_err_1.plot(controllerState_time,controllerState_xPerr, label=bag_name)
	ax_err_1.set(xlabel='Time [s]', ylabel='x position error [m]')
	ax_err_1.set_ylim(-0.25,0.25)
	ax_err_1.legend(loc='best')
	ax_err_2.plot(controllerState_time,controllerState_yPerr, label=bag_name)
	ax_err_2.set(xlabel='Time [s]', ylabel='y position error [m]')
	ax_err_2.set_ylim(-0.25,0.25)
	ax_err_2.legend(loc='best')	

ax_traj.plot(controllerState_xref,controllerState_yref,'k', label="reference")
ax_traj.legend(loc='best')

ax_pose_1.plot(controllerState_time,controllerState_xref, 'k--', label="x ref")
ax_pose_1.legend(loc='best')
ax_pose_2.plot(controllerState_time,controllerState_yref, 'k--', label="y ref")
ax_pose_2.legend(loc='best')

plt.show()
	
	
