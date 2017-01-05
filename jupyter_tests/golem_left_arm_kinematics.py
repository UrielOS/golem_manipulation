#!/usr/bin/env python

import numpy as np
from math import sqrt, atan2, sin, cos, pi

# Golem links dimensions
L1z = 1.283  # distance from floor to chest
L1x = 0.1225  # distance from chest to shoulder10
L2x = 0.212  # distance from shoulder10 to shoulder11
L41 = 0.249  # L41 and L42 are legs of the right triangle formed by arm
L42 = 0.060
L52 = 0.200  # L5x and L52 are legs of the right triangle formed by forearm
L5x = 0.034
L61 = 0.20  # distance from forearm end to gripper center

# auxiliary measures for arm and forearm
L4x = sqrt(L41 ** 2 + L42 ** 2)
rho = atan2(L42, L41)
L6x = L52 + L61
eta = atan2(L52, L5x) - pi / 4

min_th1, max_th1 = -1.00, 1.00
min_th2, max_th2 = -1.14, 1.75
min_th3, max_th3 = -3.00, 3.00
min_th4, max_th4 = -1.55, 1.55
min_th5, max_th5 = -1.67, 1.67
min_th6, max_th6 = -1.50, 1.50


# Homogeneous matrix created from Denavit-Hartenberg parameters
def dh(alpha, a, d, theta):
    homo_matrix = np.array([
        [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
        [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
        [0.0, sin(alpha), cos(alpha), d],
        [0.0, 0.0, 0.0, 1.0]
    ])
    return homo_matrix


# This function keeps an angle between -pi and +pi
def rad_limit(theta):
    phi = theta
    if theta < -pi:
        phi += 2 * pi
    if theta > pi:
        phi -= 2 * pi
    return phi


def forward_kinematics(t1, t2, t3, t4, t5, t6):
    tol = 6

    # A01 -> Homogeneous transformation from base1 to base0
    A0p = dh(0, 0.162, 0, 0)
    A01 = dh(0, -L1x, L1z, t1 - pi / 2)
    A12 = dh(0, -L2x, 0, t2)
    A23 = dh(pi / 2, 0, 0, t3 + pi / 2)  # pi/2 increment gives initial position
    A34 = dh(0, L4x, 0, t4 - pi / 2 + rho)  # pi/2 decrease gives initial position
    A45 = dh(-pi / 2, L5x, 0, t5 - rho)
    A56 = dh(pi / 2, 0, L6x, t6 - pi / 2)  # pi/2 decrease gives initial position

    # Matrix compositions
    A00 = A0p.dot(A01)
    A02 = A00.dot(A12)
    A03 = A02.dot(A23)
    A04 = A03.dot(A34)
    A05 = A04.dot(A45)
    A06 = A05.dot(A56)
    A06 = A06.dot(dh(0, 0, 0, pi / 2))

    # End effector coordinates
    x6 = A06[0, 3]
    y6 = A06[1, 3]
    z6 = A06[2, 3]
    yaw6 = rad_limit(t6)
    pitch6 = rad_limit(-t4 - t5)
    roll6 = rad_limit(t1 + t2 + t3)

    return round(x6, tol), round(y6, tol), round(z6, tol), round(yaw6, tol), round(pitch6, tol), round(roll6, 4)


def inverse_kinematics(x, y, z, yaw, pitch, roll):

    i = 1
    etol = 0.001  # error tolerance of solution
    rtol = 6  # round decimals
    stol = 6  # solution decimals
    solution = []

    # robot dimensions
    h = L1z
    m = 0.162
    l1 = L1x
    l2 = L2x
    l4 = L4x
    l5 = L5x
    l6 = L6x

    for j1 in range(2):
        for j2 in range(2):
            for j3 in range(2):
                try:
                    # theta6 computation
                    t6 = rad_limit(yaw)

                    # theta4 calculation
                    c4 = (1 / l4) * (h - z - l5 * cos(-pitch) + l6 * sin(-pitch))
                    s4 = ((-1) ** j1) * sqrt(1 - c4 ** 2)
                    t4 = rad_limit(atan2(s4, c4) - rho)
                    if not max_th4 > t4 > min_th4:
                        i += 1
                        continue

                    # theta5 calculation
                    t5 = rad_limit(-pitch - t4)
                    if not max_th5 > t5 > min_th5:
                        i += 1
                        continue

                    # theta1 calculation
                    l3 = l4 * sin(t4 + rho) + l5 * sin(t4 + t5) + l6 * cos(t4 + t5)

                    xp = -x + m + l3 * cos(roll)
                    yp = y - l3 * sin(roll)
                    a = 2 * yp * l1
                    b = 2 * xp * l1
                    c = xp ** 2 + yp ** 2 + l1 ** 2 - l2 ** 2

                    c1 = (2 * a * c - ((-1) ** j2) *
                          (sqrt((2 * a * c) ** 2 - 4 * (a ** 2 + b ** 2) * (c ** 2 - b ** 2)))) / \
                         (2 * (a ** 2 + b ** 2))

                    s1 = ((-1) ** j3) * sqrt(1 - c1 ** 2)
                    t1 = rad_limit(atan2(s1, c1))
                    if not max_th1 > t1 > min_th1:
                        i += 1
                        continue

                    # theta2 calculation
                    t12 = atan2(xp - l1 * s1, yp - l1 * c1)
                    t2 = rad_limit(t12 - t1)
                    if not max_th2 > t2 > min_th2:
                        i += 1
                        continue

                    # theta3 calculation
                    t3 = rad_limit(roll - t1 - t2)
                    if not max_th3 > t3 > min_th3:
                        i += 1
                        continue

                    signs = (j1, j2, j3)

                    # merge solution, obtain direct kinematics and test if it match the input
                    current_sol = (round(t1, stol), round(t2, stol), round(t3, stol), round(t4, stol), round(t5, stol),
                                   round(t6, stol))
                    dk = forward_kinematics(*current_sol)
                    test = (round(x, rtol), round(y, rtol), round(z, rtol), round(yaw, rtol), round(pitch, rtol),
                            round(roll, rtol))

                    if abs(dk[0] - test[0]) <= etol:
                        if abs(dk[1] - test[1]) <= etol:
                            if abs(dk[2] - test[2]) <= etol:
                                if abs(dk[3] - test[3]) <= etol:
                                    if abs(dk[4] - test[4]) <= etol:
                                        solution.append(current_sol)
                                        print 'sol', i, '|', signs, '|', current_sol

                except ValueError:
                    pass
                i += 1
    return solution


def ik_position(x, y, z):
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    inc_roll = 0.01  # 0.01rad = 0.57grad
    inc_pitch = 0.01  # 0.035rad = 2.01grad
    sweep_roll = 300
    sweep_pitch = 300
    count = 0

    for i in range(sweep_roll):
        for m in range(2):
            if roll == 0.0 and m == 1:
                count += sweep_pitch * 2
                continue
            roll *= (-1) ** m
            pitch = 0.0
            for j in range(sweep_pitch):
                for n in range(2):
                    if pitch == 0.0 and n == 1:
                        count += 1
                        continue
                    pitch *= (-1) ** n
                    ik_sol = inverse_kinematics(x, y, z, yaw, pitch, roll)
                    if ik_sol:
                        print 'Solution was found in', count, 'iterations:'
                        print 'pitch=', pitch, ' roll=', roll
                        return ik_sol
                    count += 1
                pitch = abs(pitch) + inc_pitch
        roll = abs(roll) + inc_roll

    print count, 'iterations were performed.'
    print 'pitch=', pitch, ' roll=', roll
    print 'No solution was found.'

    return ()
