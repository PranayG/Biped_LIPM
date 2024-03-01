import numpy as np
import time
import accelerometer 



g = -9.81            # Acceleration due to gravity (m/s^2)
L = 0.290            # Length of pendulum (m)
m = 6                # Mass of pendulum (kg)
b = 0.5              # Damping coefficient
Kp = 200             # Proportional gain
Kd = 0.1             # Derivative gain
#Ki = 0.1            # Integral gain

device=accelerometer.Accelerometer()

# Initial conditions
theta0 = 17 * np.pi / 18      # Initial angle (radians)
theta_dot0 = 0                # Initial angular velocity (radians/s)

# Simulation parameters
dt = 0.1                        # Time step (s)
t = np.arange(0,10,0.1)
theta = np.zeros(len(t))
theta_dot = np.zeros(len(t))
u = np.zeros(len(t))

# theta_dot_imu_x = tuple(device.readGyroData())[0]
# theta_dot_imu_x = theta_dot_imu[0]
theta_dot_imu =[0]

theta[0] = theta0
theta_dot[0] = theta_dot0
theta_dot_des = 0
theta_des = np.pi
# theta_dot_imu[0] = 0
for i in range(1,101):
    

    theta_dot_dot = g / L * np.sin(theta[i - 1]) + b / (m * L ** 2) * theta_dot[i - 1] + u[i - 1] / (m * L ** 2)

    theta_dot[i] = theta_dot[i - 1] + dt * theta_dot_dot
    theta[i] = theta[i - 1] + dt * theta_dot[i]

    # Position control
    ang_ctrl = Kp * (theta_des - theta[i - 1]) + Kd * (theta_dot_des - theta_dot[i - 1])

    # Supply motor ang_ctrl values
    # u[i] = Torque [Since dealing with angles] [If position control then u[i] would be Force]
    # Check (IMU angl vel values) a

    theta_dot_imu_x = tuple(device.readGyroData())[0]
    # if -0.05 < theta_dot_imu_x < 0.05 :
    theta_dot_imu.append(theta_dot_imu_x)

    u[i] = m * L ** 2 * (theta_dot_imu[i] - theta_dot_imu[i - 1]) / dt

    print(u[i])

    time.sleep(0.5)
    # print(len(u[i]))

# print(theta_dot_imu_x)
