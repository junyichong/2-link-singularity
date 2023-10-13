import numpy as np
import csv

def calculate_joint_angles(x, y):
    L1 = lengthBoom
    L2 = lengthArm

    # Calculate joint angles
    A = x
    B = y
    C1 = (x * x + y * y + L1 * L1 - L2 * L2) / (2 * L1)
    C2 = (x * x + y * y - L1 * L1 + L2 * L2) / (2 * L2)

    theta1 = np.arctan2(B, A)
    alpha = np.arctan2(np.sqrt(A * A + B * B - C1 * C1), C1)
    beta = np.arctan2(np.sqrt(A * A + B * B - C2 * C2), C2)

    theta1_1 = theta1 + alpha
    theta1_2 = theta1 - alpha

    theta12_1 = theta1 + beta
    theta12_2 = theta1 - beta

    theta2_1 = theta12_1 - theta1_1
    theta2_2 = theta12_1 - theta1_2
    theta2_3 = theta12_2 - theta1_1
    theta2_4 = theta12_2 - theta1_2

    return np.degrees(theta1_1), np.degrees(theta1_2), np.degrees(theta2_1), np.degrees(theta2_2), np.degrees(theta2_3), np.degrees(theta2_4)

def calculate_joint_speed(theta1, theta2, vx, vy):
    L1 = lengthBoom
    L2 = lengthArm

    # Initialize the Jacobian matrix
    J = np.array([
        [-L1 * np.sin(theta1) - L2 * np.sin(theta1 + theta2), -L2 * np.sin(theta1 + theta2)],
        [L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2), L2 * np.cos(theta1 + theta2)]
    ])

    # Create a vector representing the end-effector velocity
    V = np.array([vx, vy])

    # Calculate the determinant of the Jacobian matrix
    detJ = np.linalg.det(J)

    # Check if the determinant is zero (singularity)
    if np.abs(detJ) < 1e-6:
        omega1 = 100000
        omega2 = 100000
        print("Singularity detected. Cannot calculate joint speeds.")
    else:
        # Calculate the inverse Jacobian matrix
        JInverse = np.linalg.inv(J)

        # Calculate the joint speeds (angular velocities)
        omega = np.dot(JInverse, V)
        omega1, omega2 = omega[0], omega[1]
        omega1 = omega1 * 180.0 / np.pi
        omega2 = omega2 * 180.0 / np.pi

    return omega1, omega2

# Define the lengths of the robot arm links
lengthBoom = 3.526
lengthArm = 1.474

# Specify the output CSV file name
output_csv_file = 'data/singularity_data_4.csv'

# Create a list to store singularity data
singularity_data = []

# Initail angle values
angleArm = -40 * np.pi / 180.0
angleBoom = 50 * np.pi / 180.0

# Initial end-effector position
x = lengthBoom * np.cos(angleBoom) + lengthArm * np.cos(angleBoom + angleArm)
y = lengthBoom * np.sin(angleBoom) + lengthArm * np.sin(angleBoom + angleArm)

theta1_1, theta1_2, theta2_1, theta2_2, theta2_3, theta2_4 = calculate_joint_angles(x, y)
print(f"Theta1: {theta1_1}, theta2: {theta2_3}")


# Define the range of theta1 and theta2 values
theta1_values = np.arange(-1, 51, 0.1) 
theta2_values = np.arange(-39, -131, -0.1) 

# Define the range of vx and vy values
vx_values = np.arange(-1, 1, 0.1)
vy_values = np.arange(-1, 1, 0.1) 

# Iterate through the values
for theta1 in theta1_values:
    for theta2 in theta2_values:
        for vx in vx_values:
            for vy in vy_values:

                # Calculate joint speeds for the current theta1, theta2, vx, vy
                omega1, omega2 = calculate_joint_speed(np.radians(theta1), np.radians(theta2), vx, vy)

                if np.abs(omega1) > 200 or np.abs(omega2) > 200:
                    # Singularity detected, store data in the list
                    singularity_data.append([vx, vy, theta1, theta2, omega1, omega2])
                    continue

                # Print or use the results as needed
                # print(f"Theta1: {theta1}, Theta2: {theta2}, Omega1: {omega1}, Omega2: {omega2}")


# Write the singularity data to a CSV file
with open(output_csv_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Vx", "Vy", "Theta1", "Theta2", "Omega1", "Omega2"])
    writer.writerows(singularity_data)

print("Data has been written to", output_csv_file)