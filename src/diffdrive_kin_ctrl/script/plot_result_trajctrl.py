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
controllerState_angularvelocity_r = []
controllerState_angularvelocity_l = []
controllerState_xPerr = []
controllerState_yPerr = []

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
        controllerState_angularvelocity_r.append(msg.data[9])
        controllerState_angularvelocity_l.append(msg.data[10])
        controllerState_xPerr.append(msg.data[3]-msg.data[5])
        controllerState_yPerr.append(msg.data[4]-msg.data[6])

bag.close()

# Plot data
plt.figure(1)
plt.plot(vehicleState_x,vehicleState_y, label="actual")
plt.plot(vehicleState_x[0],vehicleState_y[0],'ro')
plt.plot(vehicleState_x[len(vehicleState_x)-1],vehicleState_y[len(vehicleState_x)-1],'rx')
plt.plot(controllerState_xref,controllerState_yref,'g', label="reference")

plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.legend(loc="best")

plt.figure(2)
plt.subplot(211)
plt.plot(vehicleState_time,vehicleState_angularvelocity_r)
plt.xlabel("Time [s]")
plt.ylabel("Right angular velocity [rad/s]")
plt.subplot(212)
plt.plot(vehicleState_time,vehicleState_angularvelocity_l)
plt.xlabel("Time [s]")
plt.ylabel("Left angular velocity [rad/s]")

plt.figure(3)
plt.subplot(311)
plt.plot(vehicleState_time,vehicleState_x, label="x act")
plt.plot(controllerState_time,controllerState_xref, 'r--', label="x ref")
plt.xlabel("Time [s]")
plt.ylabel("x [m]")
plt.legend(loc='best')
plt.subplot(312)
plt.plot(vehicleState_time,vehicleState_y, label="y act")
plt.plot(controllerState_time,controllerState_yref, 'r--', label="y ref")
plt.xlabel("Time [s]")
plt.ylabel("y [m]")
plt.legend(loc='best')
plt.subplot(313)
plt.plot(vehicleState_time,vehicleState_theta)
plt.xlabel("Time [s]")
plt.ylabel("theta [rad]")

plt.figure(4)
plt.subplot(211)
plt.plot(controllerState_time,controllerState_xPerr)
plt.xlabel("Time [s]")
plt.ylabel("x position error [m]")
plt.subplot(212)
plt.plot(controllerState_time,controllerState_yPerr)
plt.xlabel("Time [s]")
plt.ylabel("y position error [m]")

plt.show()

